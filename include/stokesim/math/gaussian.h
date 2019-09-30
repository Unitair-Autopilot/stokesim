/** @file
Class for sampling from a Gaussian (normal) distribution on Euclidean R^n.
This is modified from https://github.com/beniz/eigenmvn/blob/master/eigenmvn.h
Here is the original license:

    * Copyright (c) 2014 by Emmanuel Benazera beniz@droidnik.fr, All rights reserved.
    * This library is free software; you can redistribute it and/or
    * modify it under the terms of the GNU Lesser General Public
    * License as published by the Free Software Foundation; either
    * version 3.0 of the License, or (at your option) any later version.
    * This library is distributed in the hope that it will be useful,
    * but WITHOUT ANY WARRANTY; without even the implied warranty of
    * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    * Lesser General Public License for more details.
    * You should have received a copy of the GNU Lesser General Public
    * License along with this library.
*/
#pragma once
#include <random>
#include "stokesim/common.h"

////////////////////////////////////////////////// CONFIGURE EIGEN RANDOM

/// @cond HIDDEN_SYMBOLS
namespace Eigen {
namespace internal {

    template <typename Scalar>
    struct scalar_normal_dist_op {
        static std::mt19937 rng; // the uniform pseudo-random algorithm
        mutable std::normal_distribution<Scalar> norm; // gaussian combinator

        EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

        template <typename Index>
        inline Scalar const operator()(Index, Index=0) const {
            return norm(rng);
        }

        inline void seed(uint64_t const& s) {
            rng.seed(s);
        }
    };

    template <typename Scalar>
    std::mt19937 scalar_normal_dist_op<Scalar>::rng;
        
    template <typename Scalar>
    struct functor_traits<scalar_normal_dist_op<Scalar>> {
        enum {
            Cost = 50 * NumTraits<Scalar>::MulCost,
            PacketAccess = false,
            IsRepeatable = false
        };
    };

} // namespace internal
} // namespace Eigen
/// @endcond

////////////////////////////////////////////////// MAIN CLASS

namespace stokesim {
namespace math {

/// Core random number generator
static std::mt19937 RNG;

/// Finds the eigen-decomposition of a covariance matrix and stores it for sampling from a multivariate normal
class Gaussian {
    Eigen::MatrixXd _covar;
    Eigen::MatrixXd _transform;
    Eigen::VectorXd _mean;
    Eigen::internal::scalar_normal_dist_op<Escal> randN; // Gaussian functor
    bool _use_cholesky;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> _eigenSolver; // drawback: this creates a useless eigenSolver when using Cholesky decomposition, but it yields access to eigenvalues and vectors

public:
    Gaussian(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar,
             uint64_t const& seed=time(0), bool const use_cholesky=false) :
        _use_cholesky(use_cholesky) {
        randN.seed(seed);
        setMean(mean);
        setCovar(covar);
    }

    inline void setMean(Eigen::VectorXd const& mean) {
        _mean = mean;
    }
    
    inline void setCovar(Eigen::MatrixXd const& covar) {
        _covar = covar;

        // Assuming that we'll be using this repeatedly,
        // compute the transformation matrix that will
        // be applied to unit-variance independent normals
        if(_use_cholesky) {
            Eigen::LLT<Eigen::MatrixXd> cholSolver(_covar);

            // We can only use the cholesky decomposition if 
            // the covariance matrix is symmetric, pos-definite.
            // But a covariance matrix might be pos-semi-definite.
            // In that case, we'll go to an EigenSolver
            if(cholSolver.info() == Eigen::Success) {
                // Use cholesky solver
                _transform = cholSolver.matrixL();
            } else {
                throw std::runtime_error("Cholesky decomposition failed in instance of class Gaussian.");
            }
        } else {
            _eigenSolver = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(_covar);
            _transform = _eigenSolver.eigenvectors()*_eigenSolver.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
        }
    }

    // Draw nn samples from the gaussian and return them
    // as columns in a Dynamic by nn matrix
    inline Eigen::Matrix<Escal, Eigen::Dynamic, -1> samples(int nn) {
        return (_transform * Eigen::Matrix<Escal, Eigen::Dynamic, -1>::NullaryExpr(_covar.rows(), nn, randN)).colwise() + _mean;
    }
};

//////////////////////////////////////////////////

} // namespace math
} // namespace stokesim
