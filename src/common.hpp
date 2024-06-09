#ifndef _COMMON_HPP
#define _COMMON_HPP

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/time.h>
#include <math.h>

using namespace std;

#define TIME_ITV(start, end)	((end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec))
#define RECORD_TIME(tv)				(gettimeofday(&(tv), NULL))

#define LIMIT(x,min,max) 	((x) = ((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x) )))
#define LIMIT_R(x,min,max) 	((x) < (min)  ? (min) : ((x) > (max) ? (max) : (x)))

template <typename T>
class Vec2 {
public:
    T x_, y_;
    
    Vec2(): x_(0.0), y_(0.0) {}
    Vec2(const T x, const T y): x_(x), y_(y) {}
    Vec2<T> operator+(const Vec2<T> &v) const {return Vec2<T>(x_ + v.x_, y_ + v.y_);}
    Vec2<T> operator-(const Vec2<T> &v) const {return Vec2<T>(x_ - v.x_, y_ - v.y_);}
    Vec2<T> operator*(const T v) const {return Vec2<T>(x_ * v, y_ * v);}
    friend Vec2<T> operator*(const T v, const Vec2<T> a) {return Vec2<T>(a.x_ * v, a.y_ * v);}
    T operator*(const Vec2<T> v) const {return x_ * v.x_ + y_ * v.y_;}
    Vec2<T> operator/(const T v) const {return Vec2<T>(x_ / v, y_ / v);}
    bool operator==(const Vec2 &v) const {return x_ == v.x_ && y_ == v.y_;}
    double mod() const {return sqrt(x_ * x_ + y_ * y_);}
    double square_norm() const {return x_ * x_ + y_ * y_;}
    double ang() const {return atan2(y_, x_);}
};

#define Vec2d Vec2<double>
#define Vec2f Vec2<float>
#define Vec2i Vec2<int>

inline double fabs_(const double &a) {
    if (a > 0)
        return a;
    else
        return -a;
}

class VectorXd
{
private:
    double *data_;
    int rows_;
public:
    VectorXd(int rows) : data_(new double[rows]), rows_(rows) {
        for (int i = 0; i < rows; i++) {
            data_[i] = 0.0;
        }
    }
    // VectorXd(const VectorXd &a) : data_(new double[a.rows()]), rows_(a.rows()) {
    //     for (int i = 0; i < rows_; i++) {
    //         data_[i] = a.data_[i];
    //     }
    // }
    VectorXd(const VectorXd &a) : data_(nullptr), rows_(0) {
        *this = a;
    }
    VectorXd() : data_(nullptr), rows_(0) {}
    ~VectorXd() {
        if (data_) {
            delete []data_;
            data_ = nullptr;
        }
    }
    int rows() const {return rows_;}
    int size() const {return rows_;}
    static VectorXd Zero(int rows) {
        VectorXd z(rows);
        return z;
    }
    double &operator()(int index) {return data_[index];}
    double operator()(int index) const {return data_[index];}
    VectorXd &operator=(const VectorXd &a) {
        if (this == &a) {
            return *this;
        }
        if (rows() != a.rows()) {
            if (data_) {
                delete []data_;
                data_ = nullptr;
            }
            data_ = new double[a.rows()];
            rows_ = a.rows();
        }
        for (int i = 0; i < rows(); i++) {
            data_[i] = a(i);
        }
        return *this;
    }
    VectorXd operator-() {
        VectorXd ret(this->rows());
        for (int i = 0; i < this->rows(); i++) {
            ret(i) = -this->operator()(i);
        }
        return ret;
    }
    VectorXd operator-(const VectorXd &a) const {
        VectorXd ret(this->rows());
        for (int i = 0; i < this->rows(); i++) {
            ret(i) = this->operator()(i) - a(i);
        }
        return ret;
    }
    VectorXd &operator-=(const VectorXd &a) {
        for (int i = 0; i < this->rows(); i++) {
            this->operator()(i) -= a(i);
        }
        return *this;
    }
    VectorXd operator+(const VectorXd &a) const {
        VectorXd ret(this->rows());
        for (int i = 0; i < this->rows(); i++) {
            ret(i) = this->operator()(i) + a(i);
        }
        return ret;
    }
    VectorXd &operator+=(const VectorXd &a) {
        for (int i = 0; i < this->rows(); i++) {
            this->operator()(i) += a(i);
        }
        return *this;
    }
    VectorXd operator*(const double a) const {
        VectorXd ret = *this;
        for (int i = 0; i < this->rows(); i++) {
            ret(i) *= a;
        }
        return ret;
    }
    VectorXd &operator*=(const double a) {
        for (int i = 0; i < this->rows(); i++) {
            (*this)(i) *= a;
        }
        return *this;
    }
    friend VectorXd operator*(const double a, const VectorXd &b) {
        VectorXd ret = b;
        for (int i = 0; i < ret.rows(); i++) {
            ret(i) *= a;
        }
        return ret;
    }
    VectorXd abs() {
        VectorXd ret(this->rows());
        for (int i = 0; i < this->rows(); i++) {
            ret(i) = fabs_(this->operator()(i));
        }
        return ret;
    }
    double max_coeff() {
        double max = -INFINITY;
        for (int i = 0; i < this->rows(); i++) {
            if (this->operator()(i) > max) {
                max = this->operator()(i);
            }
        }
        return max;
    }
    double norm() {
        double n = 0.0;
        for (int i = 0; i < this->rows(); i++) {
            n += this->operator()(i) * this->operator()(i);
        }
        return sqrt(n);
    }
    double squared_norm() {
        double n = 0.0;
        for (int i = 0; i < this->rows(); i++) {
            n += this->operator()(i) * this->operator()(i);
        }
        return n;
    }
    double dot(const VectorXd &a) const {
        double ret = 0.0;
        for (int i = 0; i < this->rows(); i++) {
            ret += (*this)(i) * a(i);
        }
        return ret;
    }
    
};

class MatrixXd
{
private:
    VectorXd *data_;
    int rows_, cols_;
public:
    MatrixXd(int rows, int cols) : rows_(rows), cols_(cols) {
        data_ = new VectorXd[cols];
        for (int c = 0; c < cols; c++) {
            data_[c] = VectorXd(rows);
        }
    };
    ~MatrixXd() {
        if (data_ != nullptr) {
            delete []data_;
            data_ = nullptr;
        }
    }
    VectorXd &col(int index) {
        return data_[index];
    }
};

#endif