#include "optimizer.h"
#include "FaceModel.h"

Optimizer::Optimizer()
{
    //initialize model-morph bases
    model = FaceModel::getInstance();    
    u_id = model->getUIdentity();
    u_ex = model->getUExpression();
}

Optimizer::~Optimizer()
{
    cout << "optimizer destructor" << endl;
    M.clear();
}

void Optimizer::setPointIndices(const vector<int>&point_indices)
 {
     int index = 0;
     const int exr_size = model->getExpSize();
     const int id_size = model->getIdSize();
     Mat_<double> Mi(3,exr_size*id_size);

     M.clear();

     for(unsigned int i=0;i<point_indices.size();++i)
     {
         index = point_indices[i];
         //if not already in the map
         if(M.find(index) == M.end())
         {
             Mi = model->coreSubmatrix(index*3 , index*3 + 2 );             
             M[index] = Mi;
         }
     }
 }

