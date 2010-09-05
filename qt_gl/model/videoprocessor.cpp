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
}

VideoProcessor::VideoProcessor(const vector<cv::Point2f> &featurePoints, const vector<cv::Mat> &frameData,
                               const cv::Mat &cameraMatrix, const cv::Mat &lensDist, const unsigned int fmax,
                               const unsigned int imax) : FRAME_MAX(fmax), ITER_MAX(imax)
{
    fPoints.assign(featurePoints.begin(),featurePoints.end());
    fData.assign(frameData.begin(),frameData.end());
    this->cameraMatrix = cameraMatrix;
    this->lensDist = lensDist;

    flowEngine = new OpticalFlowEngine();
    paramOptimizer = new NNLSOptimizer();
    poseEstimator = new PoseEstimator();
}

VideoProcessor::VideoProcessor(NNLSOptimizer *paramOptimizer, OpticalFlowEngine *flowEngine,
                               const unsigned int fmax, const unsigned int imax) : FRAME_MAX(fmax), ITER_MAX(imax)
{
    this->flowEngine = flowEngine;
    this->paramOptimizer = paramOptimizer;
    this->poseEstimator = new PoseEstimator();
}


void VideoProcessor::run()
{
    this->processVideo(fPoints,fData,cameraMatrix,lensDist,frameTranslation,frameRotation,generatedPoints,
                       vector_weights_exp,vector_weights_id);    
}

void VideoProcessor::setFlowEngine(OpticalFlowEngine *flowEngine)
{
    this->flowEngine = flowEngine;
}

void VideoProcessor::setOptimizer(NNLSOptimizer *paramOptimizer)
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
        cout << vector_weights_id[0][i] << endl;
        w_id[i] = vector_weights_id[0][i];
    }
    cout << " xx " << endl;
    for(int i=0;i<EXP;i++)
    {
        cout << vector_weights_exp[frameIndex][i] << endl;
        w_exp[i] = vector_weights_exp[frameIndex][i];
    }

    face_ptr->interpolate(w_id,w_exp);
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

void VideoProcessor::processVideo2(const vector<cv::Point2f> &inputPoints, const vector<cv::Mat> &frameData,
                                  const Mat &cameraMatrix, const Mat &lensDist,
                                  vector<cv::Mat> &frameTranslation, vector<cv::Mat> &frameRotation,
                                  vector<vector<cv::Point2f> > &generatedPoints,
                                  vector<vector<double> >&vector_weights_exp,
                                  vector<vector<double> >&vector_weights_id)
{
    vector<vector<Point2f> >featurePoints;
    vector<vector<int> >point_indices;
    vector<int> indices;
    vector<int> nextIndices;
    bool useExt = false;
    Face *face_ptr = new Face();
    double *w_id = new double[56];
    double *w_exp = new double[7];

    Mat_<double> rvec, tvec;
    vector<Point2f> imagePoints;
    vector<Point2f> newPoints;

    vector<double> weights_id;
    vector<double> weights_exp;

    //make a face guess
    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.1;
        else if(i==7)w_id[i] = 0.8;
        else if(i==20)w_id[i] = 0.1;
        else w_id[i] = 0;
    }
    w_exp[0] = 0.0;
    w_exp[1] = 0.0;
    w_exp[2] = 0.0;
    w_exp[3] = 0.2;
    w_exp[4] = 0.0;
    w_exp[5] = 0.8;
    w_exp[6] = 0.0;

    face_ptr->interpolate(w_id,w_exp);


    /**********************/
    /*calculate points in all
    frames using optical flow*/
    /**********************/
    cout << "size input points " << inputPoints.size() << endl;
    vector<Point2f> currentPoints(inputPoints.begin(),inputPoints.end());
    vector<Point2f> nextPoints;

    featurePoints.push_back(currentPoints);

    cout << "feature points bfore" << endl;
    for(unsigned int i=0;i<featurePoints[0].size();i++)
        cout << featurePoints[0][i].x << " " << featurePoints[0][i].y << endl;

    vector<Mat>::const_iterator it = frameData.begin(),
    it_last = frameData.begin(),
    it_end = frameData.end();

    //place feature point indices in indices
    indices.insert(indices.begin(),Face::fPoints,Face::fPoints+Face::fPoints_size);
    point_indices.push_back(indices);

    cout << "now first frame pose " << endl;
    //calculate the rot,trans of the intial frame
    poseEstimator->calculateTransformation(featurePoints[0],face_ptr,cameraMatrix,lensDist,indices,rvec,tvec,useExt);
    frameRotation.push_back(rvec.clone());
    frameTranslation.push_back(tvec.clone());
    useExt |= true;

    newPoints.clear();

    //add new points so that we start tracking them
    //we arent actually altering the first featurePoints[0] just the rest throught compute flow
//    poseEstimator->generatePoints(frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,420,face_ptr,newPoints,indices);
//    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());


    Utility::sampleGoodPoints(currentPoints,newPoints);
    currentPoints.clear();
    indices.clear();
    poseEstimator->reprojectInto3DUsingWeak(newPoints,frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,face_ptr,indices);

    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());
    cout << " after " << currentPoints.size() << endl;

    for(;it != it_end; ++it)
    {
        flowEngine->computeFlow(*it_last,*it,currentPoints,nextPoints,indices,nextIndices);
        it_last = it;

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

    for(unsigned int i=1;i<FRAME_MAX;++i)
    {
        /**********************/
        /*estimate pose*/
        /**********************/

        //estimate the pose parameters and place estimations into vectors rotations and translations
        //the rotations vector holds the rodrigues rotation vectors which can be converted to a rotation matrix
        poseEstimator->calculateTransformation(featurePoints[i],face_ptr,cameraMatrix,lensDist,point_indices[i],rvec,tvec,useExt);
        frameRotation.push_back(rvec.clone());
        frameTranslation.push_back(tvec.clone());
        useExt |= true;
    }

    cout << "feature points after" << endl;
    for(unsigned int i=0;i<featurePoints[0].size();i++)
        cout << featurePoints[0][i].x << " " << featurePoints[0][i].y << endl;

    for(int j=0;j<3;j++)
    {
        for(unsigned int i=0;i<FRAME_MAX;++i)
        {
            /**********************/
            /*estimate model parameters + pose*/
            /**********************/
            weights_exp.clear();
            weights_id.clear();
            paramOptimizer->estimateExpressionParameters(featurePoints[i],cameraMatrix,lensDist,face_ptr,
                                                         point_indices[i], frameRotation[i],frameTranslation[i],
                                                         weights_exp);
            vector_weights_exp.push_back(weights_exp);


            //could overload generatePoints in closedform optim to do weak perspective
            cout << "frame : " << i << endl;


        }
        paramOptimizer->estimateIdentityParameters(featurePoints,cameraMatrix,lensDist,face_ptr,
                                                   point_indices, frameRotation,frameTranslation,
                                                   vector_weights_exp,weights_id);
    }

    for(unsigned int i=0;i<FRAME_MAX;i++)
    {
        newPoints.clear();
        poseEstimator->weakPerspectiveProjectPoints(frameRotation[i],frameTranslation[i],cameraMatrix,lensDist,point_indices[i],face_ptr,newPoints);
        generatedPoints.push_back(newPoints);
    }

    delete[] w_id;
    delete[] w_exp;
}

void VideoProcessor::processVideo3(const vector<cv::Point2f> &inputPoints, const vector<cv::Mat> &frameData,
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
    double *w_id = new double[56];
    double *w_exp = new double[7];

    Mat_<double> rvec, tvec;
    vector<Point2f> imagePoints;
    vector<Point2f> newPoints;

    vector<double> weights_id;
    vector<double> weights_exp;

    //make a face guess
    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.1;
        else if(i==7)w_id[i] = 0.8;
        else if(i==20)w_id[i] = 0.1;
        else w_id[i] = 0;
    }
    w_exp[0] = 0.0;
    w_exp[1] = 0.0;
    w_exp[2] = 0.0;
    w_exp[3] = 0.2;
    w_exp[4] = 0.0;
    w_exp[5] = 0.8;
    w_exp[6] = 0.0;

    face_ptr->interpolate(w_id,w_exp);


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


    //add new points so that we start tracking them
    //we arent actually altering the first featurePoints[0] just the rest throught compute flow
//    poseEstimator->generatePoints(frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,420,face_ptr,newPoints,indices);
//    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());


    //reset current points to hold the first frame feature points
    currentPoints.clear();
    currentPoints.insert(currentPoints.begin(),inputPoints.begin(),inputPoints.end());
    indices.clear();
    indices.insert(indices.begin(),Face::fPoints,Face::fPoints+Face::fPoints_size);
    //now sample new points and add them to the feature points
    Utility::sampleGoodPoints(currentPoints,newPoints);
    poseEstimator->reprojectInto3DUsingWeak(newPoints,frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,face_ptr,indices);

    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());
    cout << " after " << currentPoints.size() << endl;

    estimationPoints.push_back(currentPoints);
    estimation_point_indices.push_back(indices);

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

    for(unsigned int i=0;i<FRAME_MAX;++i)
    {
        /**********************/
        /*estimate model parameters + pose*/
        /**********************/
        weights_exp.clear();
        weights_id.clear();
        paramOptimizer->estimateModelParameters(frameData[i],estimationPoints[i],cameraMatrix,lensDist,face_ptr,
                                                estimation_point_indices[i], frameRotation[i],frameTranslation[i],
                                                weights_id,weights_exp);
        vector_weights_exp.push_back(weights_exp);
        vector_weights_id.push_back(weights_id);

        //could overload generatePoints in closedform optim to do weak perspective
        cout << "frame : " << i << endl;
        newPoints.clear();
        poseEstimator->weakPerspectiveProjectPoints(frameRotation[i],frameTranslation[i],cameraMatrix,lensDist,estimation_point_indices[i],face_ptr,newPoints);
        generatedPoints.push_back(newPoints);
    }

    delete[] w_id;
    delete[] w_exp;
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
    double *w_id = new double[56];
    double *w_exp = new double[7];

    Mat_<double> rvec, tvec;
    vector<Point2f> imagePoints;
    vector<Point2f> newPoints;

    vector<double> weights_id;
    vector<double> weights_exp;

    //make a face guess
    for(int i=0;i<56;i++)
    {
        if(i==33)w_id[i] = 0.1;
        else if(i==7)w_id[i] = 0.8;
        else if(i==20)w_id[i] = 0.1;
        else w_id[i] = 0;
    }
    w_exp[0] = 0.0;
    w_exp[1] = 0.0;
    w_exp[2] = 0.0;
    w_exp[3] = 0.2;
    w_exp[4] = 0.0;
    w_exp[5] = 0.8;
    w_exp[6] = 0.0;

    face_ptr->interpolate(w_id,w_exp);


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


    //add new points so that we start tracking them
    //we arent actually altering the first featurePoints[0] just the rest throught compute flow
//    poseEstimator->generatePoints(frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,420,face_ptr,newPoints,indices);
//    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());


    //reset current points to hold the first frame feature points
    currentPoints.clear();
    currentPoints.insert(currentPoints.begin(),inputPoints.begin(),inputPoints.end());
    indices.clear();
    indices.insert(indices.begin(),Face::fPoints,Face::fPoints+Face::fPoints_size);
    //now sample new points and add them to the feature points
//    Utility::sampleGoodPoints(currentPoints,newPoints);
//    poseEstimator->reprojectInto3DUsingWeak(newPoints,frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,face_ptr,indices);
//
//    currentPoints.insert(currentPoints.end(),newPoints.begin(),newPoints.end());
//    cout << " after " << currentPoints.size() << endl;

    estimationPoints.push_back(currentPoints);
    estimation_point_indices.push_back(indices);

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
        vector_weights_exp.clear();
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
        weights_id.clear();
        paramOptimizer->estimateIdentityParameters(estimationPoints,cameraMatrix,lensDist,face_ptr,
                                                   estimation_point_indices, frameRotation,frameTranslation,
                                                   vector_weights_exp,weights_id);

    }
    cout << "weights size " << weights_id.size() << endl;
    vector_weights_id.push_back(weights_id);

    for(unsigned int i=0;i<FRAME_MAX;i++)
    {
        newPoints.clear();
        poseEstimator->weakPerspectiveProjectPoints(frameRotation[i],frameTranslation[i],cameraMatrix,lensDist,estimation_point_indices[i],face_ptr,newPoints);
        generatedPoints.push_back(newPoints);
    }


    delete[] w_id;
    delete[] w_exp;
}

