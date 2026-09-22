#include<iostream>
#include"matrix.hpp"
namespace rm{
    Matrix zeros(size_t n, size_t m){
         Matrix Z(n,std::vector<double>(m,0));
         return Z;
    }

    Matrix ones(size_t n, size_t m){
         Matrix O(n,std::vector<double>(m,1));
         return O;
    }

    Matrix random(size_t n, size_t m, double min, double max){
        Matrix R(n,std::vector<double>(m,/*random(min,max)*/max-min));/*随机数不太会写*/
        return R;
    }

    void show(const Matrix& matrix){
        for(size_t i;i<matrix.size();i++){
            for(size_t j;j<matrix[i].size();j++){
                std::cout<<matrix[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
    }
 
    Matrix multiply(const Matrix& matrix, double c){
        Matrix R(matrix.size(),std::vector<double>(matrix[0].size(),0.0));
        for(size_t i;i<matrix.size();i++){
            for(size_t j;j<matrix[i].size();j++){
                R[i][j]=matrix[i][j]*c;
            }
        }
        return R;
    }

    Matrix multiply(const Matrix& m1, const Matrix& m2){
           Matrix Re(m1.size(),std::vector<double>(m2[0].size(),0.0));
           for(size_t i=0;i<m1.size();i++){
            for(size_t j=0;j<m2[0].size();j++){
                for(size_t k=0;k<m2.size();k++)
                {
                    Re[i][j]=m1[i][k]*m2[k][j];
                }
            }
           }
           return Re;
    } 
    Matrix sum(const Matrix& matrix, double c){
        Matrix Re(matrix.size(),std::vector<double>(matrix[0].size(),0.0));
        for(size_t i=0;i<matrix.size();i++){
            for(size_t j=0;j<matrix[0].size();j++)
            {
                Re[i][j]=matrix[i][j]+c;
            }
        }
        return Re;
    }

    Matrix sum(const Matrix& m1, const Matrix& m2){
        Matrix Re(m1.size(),std::vector<double>(m1[0].size(),0.0));
        for(size_t i=0;i<m1.size();i++){
            for(size_t j=0;j<m1[0].size();j++){
                Re[i][j]=m1[i][j]*m2[i][j];
            }
        }
        return Re;
    }

    	Matrix transpose(const Matrix& matrix){
        Matrix Re(matrix[0].size(),std::vector<double>(matrix.size(),0.0));
        for(size_t i=0;i<matrix[0].size();i++){
            for(size_t j=0;j<matrix.size();j++){
                Re[i][j]=matrix[j][i];
            }
        }
        return Re;
    }

    Matrix minor(const Matrix& matrix, size_t n, size_t m){
        Matrix Re(matrix.size()-1,std::vector<double>(matrix[0].size()-1,0.0));
        size_t row=0,col=0;
        for(size_t i=0;i<matrix.size()-1;i++)
        {
            for(size_t j=0;j<matrix[0].size()-1;j++){
                if(i==n||j==m) continue;
                else{
                    Re[row][col]=matrix[i][j];
                    col++;
                    if(col>matrix[0].size()-1){col=0,row++;}
                }
            }
        }
        return Re;
    }

    double determinant(const Matrix& matrix){
        double sum;
        size_t n=matrix.size();
        if(n==1) sum=matrix[0][0];
        if(n==2) sum=matrix[0][0]*matrix[1][1]-matrix[0][1]*matrix[1][0];
        if(n>=3) sum=0.0; /*没招了*/
        return sum;
    }

    Matrix inverse(const Matrix& matrix){
        return matrix; /*???*/
    }

    Matrix concatenate(const Matrix& m1, const Matrix& m2, int axis){
        if(axis==0) {Matrix Re(m1.size()+m2.size(),std::vector<double>(m1.size(),0.0));
        for(size_t i=0;i<m1.size();i++){
            for(size_t j=0;j<m1[0].size();j++){
                Re[i][j]=m1[i][j];
            }
        }
        for(size_t i=0;i<m2.size();i++){
            for(size_t j=0;j<m1[0].size();j++){
                Re[i+m1.size()][j]=m2[i][j];
            }
        }
        return Re;
    }
        else
        {return m1;}/*还没想好怎么写*/
    }

    Matrix ero_swap(const Matrix& matrix, size_t r1, size_t r2){
        Matrix Re(matrix.size(),std::vector<double>(matrix[0].size(),0.0));
        std::vector<double> K;
        size_t xmin=std::min(r1,r2);
        size_t xmax=std::max(r1,r2);
        for(size_t i=0;i<matrix[xmin].size();i++){
            K.push_back(matrix[xmin][i]);
        }
        for(size_t i=0;i<matrix.size();i++){
            if(i==xmin){
                    for(size_t k=0;k<matrix[xmin].size();k++){
                        Re[i][k]=matrix[xmax][k];
                    }
                    i++;
                }
            else if(i==xmax){
                    for(size_t k=0;k<matrix[xmax].size();k++){
                        Re[i][k]=K[k];
                    }
                    i++; 
                }
            else for(size_t j=0;j<matrix[0].size();j++){
                Re[i][j]=matrix[i][j];
                }
        }
        return Re;
    }

    Matrix ero_multiply(const Matrix& matrix, size_t r, double c){
          Matrix Re(matrix.size(),std::vector<double>(matrix[0].size(),0.0));
          for(size_t i=0;i<matrix.size();i++){
            for(size_t j=0;j<matrix[0].size();j++){
                if(i==r) {Re[i][j]=c*matrix[i][j];}
                else Re[i][j]=matrix[i][j];
            }
          }
          return Re;
    }

    Matrix ero_sum(const Matrix& matrix, size_t r1, double c, size_t r2){
           Matrix Re(matrix.size(),std::vector<double>(matrix[0].size(),0.0));
           std::vector<double> T;
           for(size_t i=0;i<matrix[r1].size();i++){
            T.push_back(matrix[r1][i]*c);
        }
        for(size_t i=0;i<matrix.size();i++){
            for(size_t j=0;j<matrix[i].size();j++){
                if(i==r2) {Re[i][j]=T[j]+matrix[i][j];}
                else Re[i][j]=matrix[i][j];
            }
          }
          return Re;
    }

    Matrix upper_triangular(const Matrix& matrix){
        return matrix;
    }
}