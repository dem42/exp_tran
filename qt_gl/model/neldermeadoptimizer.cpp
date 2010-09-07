#include "neldermeadoptimizer.h"
#include "modelimageerror.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

NelderMeadOptimizer::NelderMeadOptimizer()
{
}

void NelderMeadOptimizer::estimateModelParameters(const vector<Point2f> &featurePoints,
                                                    const Mat &cameraMatrix, const Mat& lensDist,
                                                    Face* face_ptr,const vector<int> &point_indices,
                                                    const Mat &rotation, const Mat &translation,
                                                    vector<double> &weights_id, vector<double> &weights_ex)
{
    double *w_id = new double[56];
    double *w_exp = new double[7];

    Mat_<double> rmatrix;

    double min;
    ModelImageError *error;
    vector<double> id;
    vector<double> ex;

    //get weights from the current face instance
    face_ptr->getWeights(w_id,56,w_exp,7);

    /***********************/
    /* now use the newly */
    /* projected points to */
    /* calculate how to change shape */
    /***********************/
    id.assign(w_id,w_id+56);
    ex.assign(w_exp,w_exp+7);
    //sizeof(w_exp)/sizeof(double)
    //first expression
    //TODO smaller coz its too slow

    Rodrigues(rotation,rmatrix);

    for(int i=0;i<3;i++)
    {
        error = new ModelImageError(cameraMatrix,rmatrix,translation,ModelImageError::EXPRESSION);
        error->setWeights(id);

        error->setPoints(featurePoints,point_indices);
        cout << "min before exp : " << (*error)(ex) << endl;
        min=mysimplex(*error,ex,ex.size(),1);
        weights_ex = ex;
        cout << "min after exp : " << min << endl;
        delete error;

        //then identity using the expression guess


        error = new ModelImageError(cameraMatrix,rmatrix,translation,ModelImageError::IDENTITY);
        error->setWeights(weights_ex);

        error->setPoints(featurePoints,point_indices);
        cout << "min before id : " << (*error)(id) << endl;
        min=mysimplex(*error,id,id.size(),1);
        weights_id = id;
        cout << "min after id : " << min << endl;
        delete error;
    }


    for(int i=0;i<7;i++){

        cout << weights_ex[i] << " ";
    }

    for(int i=0;i<56;i++){

        cout  << weights_id[i] << " ";
    }    

    delete[] w_id;
    delete[] w_exp;
}


double NelderMeadOptimizer::mysimplex(ErrorFunction & func,vector<double>& start,
                 int n, double scale)
{
  int vh;            //index of vertex with highest value (worst vertex)
  int vs;            //index of vertex with second highest value (second worst)
  int vl;            //index of vertex with lowest value (best)
  int i,j;
  const int MAX_ITER = 1000;
  const double alpha = 1, beta = 0.5, gamma = 2, delta = 0.5;
  const double epsilon = 1.0e-6;

  double pn, qn;
  double min,fr,fe,fc,cent;        //minimum value
  double fsum, favg, s;            //variables used in the convergence test

  vector< vector<double> > simplex(n+1); //the actual simplex

  vector<double> vm(n); //coordinates of the centroid
  vector<double> vr(n); //coordinates of reflection point
  vector<double> ve(n); //coordinates of the expansion
  vector<double> vc(n); //coordinates of the contraction

  vector<double> f;  //function values of the simplex vertexes



  /*step 1: initialize simplex
   * as per Haftka et al in
   * Elements of Structural optimization
   */
  pn = scale*(sqrt(n+1)-1+n)/(n*sqrt(2));
  qn = scale*(sqrt(n+1)-1)/(n*sqrt(2));

  for(i=0;i<n;i++)
    {
      simplex[0].push_back(start[i]);
    }

  for(i=1;i<=n;i++)
    {
      for (j=0;j<n;j++)
        {
          if (i-1 == j)
            { //hmm
              simplex[i].push_back(pn + start[j]);
            }
          else
            {
              simplex[i].push_back(qn + start[j]);
            }
        }
    }
  for(i=0;i<=n;i++)
    {
      f.push_back( func(simplex[i]) );
    }

  /* print out the initial values */
  //printf("Initial Values\n");
  for (j=0;j<=n;j++) {
    //printf("%f %f %f\n",simplex[j][0],simplex[j][1],f[j]);
  }


  for(int iter=0; iter < MAX_ITER; iter++)
    {
      /* print out the value at each iteration */
      //printf("Iteration %d\n",iter);
      for (j=0;j<=n;j++) {
        //printf("%f %f %f\n",simplex[j][0],simplex[j][1],f[j]);
      }


      /*step 2: ordering, n must be at least 2 */
      /*(vs=2,1) means there will be an assignment and 1 will be returned*/
      vl = 0;
      vh = f[1]>f[2] ? (vs=2,1) : (vs=1,2);
      for (i=0;i<=n;i++)
        {
          if (f[i] <= f[vl])
            vl=i;
          if (f[i] > f[vh])
            {
              vs=vh;
              vh=i;
            }
          else if (f[i] > f[vs] && i != vh)
            vs=i;
        }


      /* print out the order */
      //printf("f[vh] = %f, f[vs] = %f, f[vl]=%f\n",f[vh],f[vs],f[vl]);
      //printf("vh = %d, vs = %d, vl=%d\n",vh,vs,vl);
      /*
       * step 3 transformation
       * consisting of reflection,expansion,contraction,shrinking
       */

      /*compute the centroid*/
      for(j=0; j < n; j++)
        {
          cent = 0.0;
          for(i=0; i <= n; i++)
            {
              if(i != vh)
                {
                  cent += simplex[i][j];
                }
            }
          vm[j] = cent / (double)n;
        }

      /*reflect*/
      for(j=0; j < n; j++)
        {
          vr[j] = vm[j] + alpha*(vm[j] - simplex[vh][j]);
        }
      fr = func(vr);
      //printf("fr = %f\n",fr);
      /*accept xr terminate iteration*/
      if(fr >= f[vl] && fr < f[vs])
        {
          for(j=0; j < n; j++)
            {
              simplex[vh][j] = vr[j];
            }
          f[vh] = fr;
          //print out
          //printf("accept xr vh = %f\n",f[vh]);

          continue;
        }
      if(fr < f[vl])
        {  /*if reflect ok then expand*/
          for(j=0; j < n; j++)
            {
              ve[j] = vm[j] + gamma*(vr[j] - vm[j]);
            }
          fe = func(ve);
          //printf("fe = %f\n",fe);
          /*accept xe and terminate iteration*/
          if(fr > fe)
            {
              for(j=0; j < n; j++)
                {
                  simplex[vh][j] = ve[j];
                }
              f[vh] = fe;
              //print out
              //printf("accept xe vh = %f\n",f[vh]);

              continue;
            }
          else
            { /*otherwise accept xr and terminate iteration*/
              for(j=0; j < n; j++)
                {
                  simplex[vh][j] = vr[j];
                }
              f[vh] = fr;
              //print out
              //printf("accept xr because xe bad vh = %f\n",f[vh]);

              continue;
            }
        }
      else if(fr >= f[vs])
        {
           /*contraction*/
          /*could just as well be else .. else if for legibility*/
          if(fr >= f[vh])
            {/* inside */
              for(j=0; j < n; j++)
                {
                  vc[j] =  vm[j] + beta*(vr[j] - vm[j]);
                }
              fc = func(vc);
              //printf("fc = %f\n",fc);
              if(fc < f[vh])
                {
                  for(j=0; j < n; j++)
                    {
                      simplex[vh][j] = vc[j];
                    }
                  f[vh] = fc;
                  //print out
                  //printf("accept xc inside vh = %f\n",f[vh]);

                  continue;
                }
            }
          else
            {/* outside */
              for(j=0; j < n; j++)
                {
                  vc[j] = vm[j] + beta*(simplex[vh][j] - vm[j]);
                }
              fc = func(vc);
              //printf("fc = %f\n",fc);
              if(fc <= fr)
                {
                  for(j=0; j < n; j++)
                    {
                      simplex[vh][j] = vc[j];
                    }

                  f[vh] = fc;
                  //print out
                  //printf("accept outside xc vh = %f\n",f[vh]);

                  continue;
                }
            }

          /*shrink for else*/
          for(i=0; i <= n; i++)
            {
              if(i != vl)
                {
                  for(j=0; j < n; j++)
                    {
                      simplex[i][j] = delta*(simplex[i][j] - simplex[vl][j]);
                      simplex[i][j] += simplex[vl][j];
                    }
                }
            }
          f[vh] = func(simplex[vh]);
          f[vs] = func(simplex[vs]);
          //print out
          //printf("shrinking vh = %f,vs = %f\n",f[vh],f[vs]);

        }

      /* test for convergence */
      fsum = 0.0;
      for (j=0;j<=n;j++) {
        fsum += f[j];
      }
      favg = fsum/(n+1);
      s = 0.0;
      for (j=0;j<=n;j++) {
        s += pow((f[j]-favg),2.0)/(n);
      }
      s = sqrt(s);
      if (s < epsilon) break;

    }
  vl = 0;
  for(i = 0; i <= n; i++)
    {
      if(f[vl] > f[i])
        {
          vl = i;
        }
    }

  //printf("The minimum was found at\n");
  for (j=0;j<n;j++) {
    //printf("%e\n",simplex[vl][j]);
    start[j] = simplex[vl][j];
  }

  min = f[vl];
  return min;
}

    void NelderMeadOptimizer::estimateExpressionParameters(const vector<Point2f> &featurePoints,
                                      const Mat &cameraMatrix, const Mat& lensDist,
                                      Face* face_ptr,const vector<int> &point_indices,
                                      const Mat &rotation, const Mat &translation,
                                      vector<double> &weights_ex)
    {
    }

    void NelderMeadOptimizer::estimateIdentityParameters(const vector<vector<Point2f> >&featurePointsVector,
                                    const Mat &cameraMatrix, const Mat& lensDist,
                                    Face* face_ptr,const vector<vector<int> >&point_indices_vector,
                                    const vector<Mat> &rotation, const vector<Mat> &translation,
                                    const vector<vector<double> > &weights_ex,
                                    vector<double> &weights_id)
    {
    }
