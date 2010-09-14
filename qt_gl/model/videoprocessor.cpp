#include "videoprocessor.h"

#include "neldermeadoptimizer.h"
#include "closedformoptimizer.h"
#include "nnlsoptimizer.h"
#include "controller/utility.h"

#include "model/exptranexception.h"

VideoProcessor::VideoProcessor(const unsigned int fmax, const unsigned int imax) : FRAME_MAX(fmax), ITER_MAX(imax)
{
    flowEngine = new OpticalFlowEngine();
    paramOptimizer = new NNLSOptimizer();
    poseEstimator = new PoseEstimator();
    threadCrashed = false;
}

VideoProcessor::VideoProcessor(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                               const cv::Mat &cameraMatrix, const cv::Mat &lensDist,
                               VideoProcessor::OptType type, double regParam,
                               const unsigned int fmax,const unsigned int imax) : FRAME_MAX(fmax), ITER_MAX(imax)
{
    fPoints.assign(featurePoints.begin(),featurePoints.end());
    fData.assign(frameData.begin(),frameData.end());
    this->cameraMatrix = cameraMatrix;
    this->lensDist = lensDist;

    if(type == OptType_INTERPOLATE)
        paramOptimizer = new NNLSOptimizer();
    else if(type == OptType_LIN_COMB)
        paramOptimizer = new ClosedFormOptimizer(regParam);
    else
        paramOptimizer = new NNLSOptimizer();

    flowEngine = new OpticalFlowEngine();
    poseEstimator = new PoseEstimator();
    threadCrashed = false;
}

VideoProcessor::VideoProcessor(Optimizer *paramOptimizer, OpticalFlowEngine *flowEngine,
                               const unsigned int fmax, const unsigned int imax) : FRAME_MAX(fmax), ITER_MAX(imax)
{
    this->flowEngine = flowEngine;
    this->paramOptimizer = paramOptimizer;
    this->poseEstimator = new PoseEstimator();
    threadCrashed = false;
}

VideoProcessor::~VideoProcessor()
{
    delete flowEngine;
    delete paramOptimizer;
    delete poseEstimator;
}

void VideoProcessor::run()
{
    try
    {
        this->processVideo(fPoints,fData,cameraMatrix,lensDist,frameTranslation,frameRotation,generatedPoints,
                           vector_weights_exp,vector_weights_id);
    }catch(cv::Exception &e)
    {        
        cerr << e.what() << endl;
        threadCrashed = true;
    }
}

bool VideoProcessor::getCrashed() const
{
    return threadCrashed;
}

void VideoProcessor::setFlowEngine(OpticalFlowEngine *flowEngine)
{
    this->flowEngine = flowEngine;
}

void VideoProcessor::setOptimizer(Optimizer *paramOptimizer)
{
    this->paramOptimizer = paramOptimizer;
}

unsigned int VideoProcessor::getFrameNum() const
{
    return FRAME_MAX;
}

void VideoProcessor::getFaceForFrame(unsigned int frameIndex, Face *face_ptr) const
{
    int ID = face_ptr->getIdNum();
    int EXP = face_ptr->getExpNum();

    double w_id[ID];
    double w_exp[EXP];
    double avg = face_ptr->getAverageDepth();

    if(frameIndex > vector_weights_exp.size())
    {
        ostringstream os;
        os << "frame index " << frameIndex << " < than number of frames computed " << vector_weights_exp.size();
        throw ExpTranException(os.str());
    }

    for(int i=0;i<ID;i++)
    {

        w_id[i] = vector_weights_id[0][i];
    }

    for(int i=0;i<EXP;i++)
    {

        w_exp[i] = vector_weights_exp[frameIndex][i];
    }

    face_ptr->setNewIdentityAndExpression(w_id,w_exp);
    face_ptr->setAverageDepth(avg);
}

void VideoProcessor::getFaceAndPoseForFrame(unsigned int frameIndex, Face *face_ptr, Mat &rot, Mat &tran) const
{
    getFaceForFrame(frameIndex,face_ptr);
    rot = frameRotation[frameIndex].clone();
    tran = frameTranslation[frameIndex].clone();
}

void VideoProcessor::getGeneratedPointsForFrame(unsigned int frameIndex, vector<Point2f> &points) const
{
    points.assign(generatedPoints[frameIndex].begin(),generatedPoints[frameIndex].end());
}


bool VideoProcessor::termination(const vector<vector<double> >&prevExp, const vector<vector<double> >&exp,
                                 const vector<double> &prevId, const vector<double> &id)
{
    double dif;
    const double exp_eps = 0.0001;
    const double id_eps = 0.001;
    for(unsigned int i=0;i<prevExp.size();i++)
    {
        dif = 0;
        cout << "ugh" << endl;
        for(unsigned int j=0;j<prevExp[i].size();j++)
        {
            dif += (prevExp[i][j] - exp[i][j])*(prevExp[i][j] - exp[i][j]);            
        }
        cout << "dif exp  " << dif << " frame " << i << endl;
        if(dif > exp_eps)
            return false;
    }
    dif = 0;
    for(unsigned int i=0;i<prevId.size();i++)
    {
        dif += (prevId[i] - id[i])*(prevId[i] - id[i]);       
    }
    cout << "dif id at  " << dif << endl;
    if(dif > id_eps)
        return false;
    return true;
}

void VideoProcessor::processVideo(const vector<cv::Point2f> &inputPoints, const vector<cv::Mat> &frameData,
                                  const Mat &cameraMatrix, const Mat &lensDist,
                                  vector<cv::Mat> &frameTranslation, vector<cv::Mat> &frameRotation,
                                  vector<vector<cv::Point2f> > &generatedPoints,
                                  vector<vector<double> >&vector_weights_exp,
                                  vector<vector<double> >&vector_weights_id)
{
    vector<vector<Point2f> >featurePoints;
    vector<vector<Point2f> >estimationPoints;

    vector<vector<int> >point_indices;
    vector<vector<int> >estimation_point_indices;

    vector<int> indices;
    vector<int> nextIndices;
    bool useExt = false;
    Face *face_ptr = new Face();
    double *w_id = new double[face_ptr->getIdNum()];
    double *w_exp = new double[face_ptr->getExpNum()];

    Mat_<double> rvec, tvec;
    vector<Point2f> imagePoints;
    vector<Point2f> newPoints;

    vector<double> weights_id;
    vector<double> weights_exp;

    vector<double> prevWeightsId;
    vector<vector<double> >vector_prev_weights_exp;

    //make a face guess
    for(int i=0;i<face_ptr->getIdNum();i++)
        w_id[i] = 1./face_ptr->getIdNum();
    for(int i=0;i<face_ptr->getExpNum();i++)
        w_exp[i] = 1./face_ptr->getExpNum();

    face_ptr->setNewIdentityAndExpression(w_id,w_exp);


    /**********************/
    /*calculate points in all
    frames using optical flow*/
    /**********************/
    cout << "size input points " << inputPoints.size() << endl;
    vector<Point2f> currentPoints(inputPoints.begin(),inputPoints.end());
    vector<Point2f> nextPoints;

    //place feature point indices in indices
    indices.insert(indices.begin(),Face::fPoints,Face::fPoints+Face::fPoints_size);
    point_indices.push_back(indices);

    featurePoints.push_back(currentPoints);


    //we start with frames 0 and 1
    for(unsigned int i=1;i<FRAME_MAX;++i)
    {
        flowEngine->computeFlow(frameData[i-1],frameData[i],currentPoints,nextPoints,indices,nextIndices);

        featurePoints.push_back(nextPoints);
        point_indices.push_back(nextIndices);
        currentPoints.clear();
        indices.clear();
        //assigns a copy of next points as the new content for currentPoints
        currentPoints = nextPoints;
        indices = nextIndices;
        nextPoints.clear();
        nextIndices.clear();
    }

    /**********************/
    /*estimate pose*/
    /**********************/
    for(unsigned int i=0;i<FRAME_MAX;++i)
    {

        //estimate the pose parameters and place estimations into vectors rotations and translations
        //the rotations vector holds the rodrigues rotation vectors which can be converted to a rotation matrix
        poseEstimator->calculateTransformation(featurePoints[i],face_ptr,cameraMatrix,lensDist,point_indices[i],rvec,tvec,useExt);
        frameRotation.push_back(rvec.clone());
        frameTranslation.push_back(tvec.clone());
        useExt |= true;
    }

    //reset current points to hold the first frame feature points
    currentPoints.clear();
    currentPoints.insert(currentPoints.begin(),inputPoints.begin(),inputPoints.end());
    indices.clear();
    indices.insert(indices.begin(),Face::fPoints,Face::fPoints+Face::fPoints_size);

    //first frame alignment computes the first face in face_ptr
//    paramOptimizer->estimateModelParameters(currentPoints,cameraMatrix,lensDist,face_ptr,indices,
//                                            frameRotation[0],frameTranslation[0],weights_id,weights_exp);



    //add new points so that we start tracking them
    //we arent actually altering the first featurePoints[0] just the rest throught compute flow
    poseEstimator->generatePoints(frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,420,face_ptr,newPoints,indices);
    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());


    //now sample new points using the first frame alignment and add them to the feature points
    Utility::sampleGoodPoints(currentPoints,newPoints);
    poseEstimator->reprojectInto3DUsingWeak(newPoints,frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,face_ptr,indices);
    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());

    newPoints.clear();
    poseEstimator->projectModelPointsInto2D(frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,face_ptr,indices,newPoints);
    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());

    cout << " after " << currentPoints.size() << endl;

    estimationPoints.push_back(currentPoints);
    estimation_point_indices.push_back(indices);

    //initialize the param optimizer with the indices (to make it faster)
    paramOptimizer->setPointIndices(indices);

    //repeat tracking of new points
    //we start with frames 0 and 1
    for(unsigned int i=1;i<FRAME_MAX;++i)
    {
        flowEngine->computeFlow(frameData[i-1],frameData[i],currentPoints,nextPoints,indices,nextIndices);

        estimationPoints.push_back(nextPoints);
        estimation_point_indices.push_back(nextIndices);
        currentPoints.clear();
        indices.clear();
        //assigns a copy of next points as the new content for currentPoints
        currentPoints = nextPoints;
        indices = nextIndices;
        nextPoints.clear();
        nextIndices.clear();
    }

    for(unsigned int j=0;j<ITER_MAX;j++)
    {
        //clear before putting new set of params in
        //vector_prev_weights_exp.assign(vector_weights_exp.begin(),vector_weights_exp.end());
        vector_prev_weights_exp.clear();
        vector<double> wexp;
        for(unsigned int k=0;k<vector_weights_exp.size();k++)
        {
            wexp.clear();
            for(unsigned int l=0;l<vector_weights_exp[k].size();l++)
                wexp.push_back(vector_weights_exp[k][l]);
            vector_prev_weights_exp.push_back(wexp);
        }

        vector_weights_exp.clear();
        vector_weights_id.clear();
        for(unsigned int i=0;i<FRAME_MAX;++i)
        {
            /**********************/
            /*estimate model parameters + pose*/
            /**********************/
            weights_exp.clear();            
            paramOptimizer->estimateExpressionParameters(estimationPoints[i],cameraMatrix,lensDist,face_ptr,
                                                         estimation_point_indices[i], frameRotation[i],frameTranslation[i],
                                                         weights_exp);
            vector_weights_exp.push_back(weights_exp);



            //could overload generatePoints in closedform optim to do weak perspective
            cout << "frame : " << i << endl;

        }
        //prevWeightsId.assign(weights_id.begin(),weights_id.end());
        prevWeightsId.clear();
        for(unsigned int k=0;k<weights_id.size();k++)
            prevWeightsId.push_back(weights_id[k]);

        cout << "weights size " << prevWeightsId.size() << endl;
        weights_id.clear();
        cout << "weights size " << prevWeightsId.size() << endl;
        paramOptimizer->estimateIdentityParameters(estimationPoints,cameraMatrix,lensDist,face_ptr,
                                                   estimation_point_indices, frameRotation,frameTranslation,
                                                   vector_weights_exp,weights_id);


        cout << "weights size " << weights_id.size() << endl;
        vector_weights_id.push_back(weights_id);

        if(j > 0 && termination(vector_prev_weights_exp,vector_weights_exp,prevWeightsId,weights_id) == true)
        {
            cout << prevWeightsId.size() << endl;
            cout << "terminating in frame " << j << endl;
            break;
        }
    }


    for(unsigned int i=0;i<FRAME_MAX;i++)
    {
        newPoints.clear();
        poseEstimator->weakPerspectiveProjectPoints(frameRotation[i],frameTranslation[i],cameraMatrix,lensDist,estimation_point_indices[i],face_ptr,newPoints);
        generatedPoints.push_back(newPoints);
    }


    delete[] w_id;
    delete[] w_exp;
}

