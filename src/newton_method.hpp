#pragma once

#include <iostream>

#include "common.hpp"

using namespace std;

typedef double (*evaluate_t)(void *instance,
                const VectorXd &x,
                VectorXd &g);

inline int line_search(
                VectorXd &x,
                double &f,
                VectorXd &g,
                double &stp,
                const VectorXd &s,
                const VectorXd &xp,
                const VectorXd &gp,
                const double stpmin,
                const double stpmax,
                const double f_dec_coeff,
                const double s_curv_coeff,
                const int max_linesearch,
                const double machine_prec,
                evaluate_t evaluate_func,
                void *instance) {
    int count = 0;
    bool brackt = false, touched = false;
    double finit, dginit, dgtest, dstest;
    double mu = 0.0, nu = stpmax;

    if (!(stp > 0.0))
    {
        return -1;
    }

    dginit = gp.dot(s);

    if (0.0 < dginit)
    {
        return -1;
    }

    finit = f;
    dgtest = f_dec_coeff * dginit;
    dstest = s_curv_coeff * dginit;

    while (true)
    {
        x = xp + stp * s;
        f = evaluate_func(instance, x, g);
        ++count;
        
        if (std::isinf(f) || std::isnan(f))
        {
            return -1;
        }
        if (f > finit + stp * dgtest)
        {
            nu = stp;
            brackt = true;
        }
        else
        {
            if (g.dot(s) < dstest)
            {
                mu = stp;
            }
            else
            {
                return count;
            }
        }
        if (max_linesearch <= count)
        {
            return -1;
        }
        if (brackt && (nu - mu) < machine_prec * nu)
        {
            return -1;
        }

        if (brackt)
        {
            stp = 0.5 * (mu + nu);
        }
        else
        {
            stp *= 2.0;
        }

        if (stp < stpmin)
        {
            return -1;
        }
        if (stp > stpmax)
        {
            if (touched)
            {
                return -1;
            }
            else
            {
                touched = true;
                stp = stpmax;
            }
        }
    }
}

inline int lbfgs_optimize(VectorXd &x,
                double &f,
                evaluate_t proc_evaluate,
                void *instance){
    int ret, i, j, k, ls, end, bound;
    double step, step_min, step_max, fx, ys, yy;
    double gnorm_inf, xnorm_inf, beta, rate, cau;
    const int n = x.size();
    const int m = 5;
    const int past = 3;
    const int max_iterations = 0;
    VectorXd xp(n);
    VectorXd g(n);
    VectorXd gp(n);
    VectorXd d(n);
    VectorXd pf(std::max(1, past));
    VectorXd lm_alpha(m);
    MatrixXd lm_s(n, m);
    MatrixXd lm_y(n, m);
    VectorXd lm_ys(m);
    
    fx = proc_evaluate(instance, x, g);
    pf(0) = fx;
    d = -g;
    gnorm_inf = g.abs().max_coeff();
    xnorm_inf = x.abs().max_coeff();
    if (gnorm_inf / std::max(1.0, xnorm_inf) <= 1e-5) {
        return 0;
    } else {
        step = 1.0 / d.norm();
        k = 1;
        end = 0;
        bound = 0;
        while (true) {
            xp = x;
            gp = g;
            step_min = 1e-20;
            step_max = 1e+20;
            step = step < step_max ? step : 0.5 * step_max;
            ls = line_search(x, fx, g, step, d, xp, gp, step_min, step_max, 1e-4, 0.9, 64, 1e-16, proc_evaluate, instance);
            if (ls < 0) {
                x = xp;
                g = gp;
                ret = ls;
                break;
            }

            gnorm_inf = g.abs().max_coeff();
            xnorm_inf = x.abs().max_coeff();
            if (gnorm_inf / std::max(1.0, xnorm_inf) < 1e-5) {
                ret = 0;
            }

            if (0 < past)
            {
                if (past <= k)
                {
                    rate = std::fabs(pf(k % past) - fx) / std::max(1.0, std::fabs(fx));

                    if (rate < 1e-6)
                    {
                        ret = 1;
                        break;
                    }
                }

                pf(k % past) = fx;
            }

            if (max_iterations != 0 && max_iterations <= k)
            {
                ret = -1;
                break;
            }

            ++k;

            lm_s.col(end) = x - xp;
            lm_y.col(end) = g - gp;
            ys = lm_y.col(end).dot(lm_s.col(end));
            yy = lm_y.col(end).squared_norm();
            lm_ys(end) = ys;
            d = -g;
            cau = lm_s.col(end).squared_norm() * gp.norm() * 1.0e-6;

            if (ys > cau)
            {
                ++bound;
                bound = m < bound ? m : bound;
                end = (end + 1) % m;

                j = end;
                for (i = 0; i < bound; ++i)
                {
                    j = (j + m - 1) % m;
                    lm_alpha(j) = lm_s.col(j).dot(d) / lm_ys(j);
                    d += (-lm_alpha(j)) * lm_y.col(j);
                }

                d *= ys / yy;

                for (i = 0; i < bound; ++i)
                {
                    beta = lm_y.col(j).dot(d) / lm_ys(j);
                    d += (lm_alpha(j) - beta) * lm_s.col(j);
                    j = (j + 1) % m; 
                }
            }
            step = 1.0;
        }
    }
    f = fx;
    return ret;
}