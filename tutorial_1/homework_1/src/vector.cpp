#include"defines.hpp"
namespace rm
{
    double length(const Vector& v)
    {
        double len = 0;
        for (auto& o : v)
        {
            len += o * o;
        }
        return sqrt(len);
    }
    double distance(const Vector& v1, const Vector& v2)
    {
        size_t s1 = v1.size(); size_t s2 = v2.size();
        if (s1 == 0 || s2 == 0)throw std::invalid_argument("dismension can't be zero.");
        else if (s1 == s2)
        {
            double result = 0;
            for (int i = 1; i <= s1; i++)
            {
                double d = v1[i - 1] - v2[i - 1];
                result += d * d;
            }
            return result;
        }
        else throw std::invalid_argument("dismension is not equal.");
    }
    double dot(const Vector& v1, const Vector& v2)
    {
        size_t s1 = v1.size(); size_t s2 = v2.size();
        if (s1 == 0 || s2 == 0)throw std::invalid_argument("dismension can't be zero.");
        else if (s1 == s2)
        {
            double result = 0;
            for (int i = 1; i <= s1; i++)
            {
                result += v1[i - 1] * v2[i - 1];
            }
            return  result;
        }
        else throw std::invalid_argument("dismension is not equal.");
    }
    Vector scale(const Vector& v, double k)
    {
        if (v.size() == 0)throw std::invalid_argument("dismension can't be zero");
        else
        {
            std::vector<double>arr(v.size());
            for (int i = 1; i <= v.size(); i++)
            {
                arr[i - 1] = arr[i - 1] * k;
            }
            return arr;
        }
    }
    Vector normalize(const Vector& v)
    {
        size_t size = v.size(); double len = length(v);
        if (size == 0)throw std::invalid_argument("dismension can't be zero");
        else if (len == 0)throw std::invalid_argument("it's a zero Vector");
        else
        {
            Vector result(v.size());
            for (int i = 1; i <= v.size(); i++)
            {
                result[i - 1] = (result[i - 1]) / len;
            }
            return result;
        }
    }
    double angleBetween(const Vector& v1, const Vector& v2)
    {
        if (v1.size() == 0 || v2.size() == 0)throw std::invalid_argument("dismension can't be zero");
        else if (v1.size() != v2.size()) throw std::invalid_argument("dismension is't equal");
        else return acos(dot(normalize(v1), normalize(v2)));
    }
    Matrix zero(size_t m, size_t n)
    {
        return Matrix(m, Vector(n, 0));
    }
    Matrix ones(size_t m, size_t n)
    {
        return Matrix(m, Vector(n, 1));
    }
    Matrix random(size_t m, size_t n, double min, double max)
    {
        Matrix result(zero(m, n));
        for (auto& v : result)
        {
            for (auto& o : v)
            {
                o = min + rand() % ((int)max - (int)min + 1);
            }
        }
        return result;
    }
    void show(const Matrix& m)
    {
        if (m.size() == 0)throw std::invalid_argument("error");
        else
        {
            for (auto& v : m)
            {
                static  size_t stand = m.size();
                if (m.size() != stand)throw std::invalid_argument("error");
                else for (auto& o : v) { std::cout <<std::fixed << std::setprecision(3)<<o << " "; }
                std::cout <<std::endl;
            }
        }
    }
    void check(const Matrix&m)
    {
        if (m.size() == 0)throw std::invalid_argument("error");
        else
        {
            for (auto& v : m)
            {
                static size_t stand = v.size();
                if (v.size() != stand)throw std::invalid_argument("error");
            }
        }
    }
    Matrix multiple(const Matrix& m,const double k)
    {
            Matrix result(m);
            for (auto& v : result)
            {
                static size_t stand = v.size();
                if (v.size() != stand)throw std::invalid_argument("error");
                else
                {
                    for (auto& o : v)
                    {
                        o = o * k;
                    }
                }
            }
            return result;
    }
    Matrix multiple(const Matrix&m,const Matrix&n) 
    {
        check(m); check(n);
        size_t a = m[0].size(); size_t b = n.size();
        if (a != b)throw std::invalid_argument("error");
        else 
        {
            Matrix result(zero(m.size(), n[0].size()));
            for (int i = 1; i <= result.size(); i++)
            {
                for (int j = 1; j <= result[0].size(); j++)
                {
                    Vector tem(n.size(), 0); int i = 1;
                    for (auto& o : n) { tem[i - 1] = o[j - 1]; i++; };
                    result[i-1][j-1]=dot(m[i-1],tem);
                }
            }
            return result;
        }
     
    }
    Matrix sum(const Matrix&m, double c)
    {
        Matrix result(m);
        check(m);
        for (auto&v:result) 
        {
            for (auto&o:v)
            {
                o = o + c;
            }
        }
        return result;
    }
    Matrix sum(const Matrix&m1,const Matrix&m2) 
    {
        check(m1); check(m2);
        if (m1.size() != m2.size() || m1[0].size() != m2[0].size())throw std::invalid_argument("error");
        else 
        {
            Matrix result(zero(m1.size(), m1[0].size()));
            for (int i = 1; i <= m1.size(); i++)
            {
                for (int j = 1; j <= m1[0].size(); j++)
                {
                    result[i - 1][j - 1] = m1[i-1][j-1] + m2[i-1][j-1];
                }
            }
        }
    }
}