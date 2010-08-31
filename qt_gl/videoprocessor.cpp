#include "videoprocessor.h"

#include "neldermeadoptimizer.h"
#include "closedformoptimizer.h"
#include "nnlsoptimizer.h"

VideoProcessor::VideoProcessor() : FRAME_MAX(10)
{
    flowEngine = new OpticalFlowEngine();
    paramOptimizer = new ClosedFormOptimizer();
}

VideoProcessor::VideoProcessor(Optimizer *paramOptimizer, OpticalFlowEngine *flowEngine) : FRAME_MAX(4)
{
    this->flowEngine = flowEngine;
    this->paramOptimizer = paramOptimizer;
}

void VideoProcessor::setFlowEngine(OpticalFlowEngine *flowEngine)
{
    this->flowEngine = flowEngine;
}

void VideoProcessor::setOptimizer(Optimizer *paramOptimizer)
{
    this->paramOptimizer = paramOptimizer;
}

void VideoProcessor::processVideo(const vector<cv::Point2f> &inputPoints, const vector<cv::Mat> &frameData,
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
    paramOptimizer->calculateTransformation(featurePoints[0],face_ptr,cameraMatrix,lensDist,indices,rvec,tvec,useExt);
    frameRotation.push_back(rvec.clone());
    frameTranslation.push_back(tvec.clone());
    useExt |= true;

    newPoints.clear();
    cout << "now new points " << endl;
    paramOptimizer->generatePoints(frameRotation[0],frameTranslation[0],cameraMatrix,lensDist,700,face_ptr,newPoints,indices);
    //add new points so that we start tracking them
    //we arent actually altering the first featurePoints[0] just the rest throught compute flow
    cout << "cur points b4 " << currentPoints.size() << endl;
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
        paramOptimizer->calculateTransformation(featurePoints[i],face_ptr,cameraMatrix,lensDist,point_indices[i],rvec,tvec,useExt);
        frameRotation.push_back(rvec.clone());
        frameTranslation.push_back(tvec.clone());
        useExt |= true;
    }

    cout << "feature points after" << endl;
    for(unsigned int i=0;i<featurePoints[0].size();i++)
        cout << featurePoints[0][i].x << " " << featurePoints[0][i].y << endl;

    for(unsigned int i=0;i<FRAME_MAX;++i)
    {
        /**********************/
        /*estimate model parameters + pose*/
        /**********************/
        weights_exp.clear();
        weights_id.clear();
        paramOptimizer->estimateModelParameters(frameData[i],featurePoints[i],cameraMatrix,lensDist,face_ptr,
                                                point_indices[i], frameRotation[i],frameTranslation[i],
                                                weights_id,weights_exp);
        vector_weights_exp.push_back(weights_exp);
        vector_weights_id.push_back(weights_id);

        //could overload generatePoints in closedform optim to do weak perspective
        cout << "frame : " << i << endl;
        newPoints.clear();
        paramOptimizer->weakPerspectiveProjectPoints(frameRotation[i],frameTranslation[i],cameraMatrix,lensDist,point_indices[i],face_ptr,newPoints);
        generatedPoints.push_back(newPoints);
    }

    delete[] w_id;
    delete[] w_exp;
}
