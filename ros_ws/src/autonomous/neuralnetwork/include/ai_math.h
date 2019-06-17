#pragma once

#include <algorithm>
#include <cstdlib>
#include <vector>
#include <cassert>

// clang-format off
#include "floatfann.h"
#include "fann_cpp.h"
// clang-format on

namespace ai_math
{
    using namespace std;

    typedef std::vector<fann_type> NetVector;

    // ################################################################
    // #    create vectors
    // ################################################################

    // returns a vector with random values between min and max
    inline NetVector random_uniform_distribution(size_t size, fann_type min = -1, fann_type max = 1)
    {
        assert(min <= max);
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<fann_type> dis(min, max);
        NetVector vec(size);
        for (NetVector::size_type i = 0; i < size; i++)
        {
            vec[i] = dis(gen);
        }

        return vec;
    }

    // returns a vector with random values between min and max
    inline NetVector random_normal_distribution(size_t size, double mean = 0, double stddev = 0.5)
    {
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::normal_distribution<fann_type> dis(mean, stddev);
        NetVector vec(size);
        for (NetVector::size_type i = 0; i < size; i++)
        {
            vec[i] = dis(gen);
        }

        return vec;
    }

    // creates a vector with n ones and (size - n) zeros
    inline NetVector random_binary_mutation(size_t size, size_t number_of_trues = 1)
    {
        assert(number_of_trues <= size);
        std::vector<bool> vec_boolean(size);
        for (NetVector::size_type i = 0; i < size; i++)
        {
            vec_boolean[i] = false;
        }

        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> dis(0, size - 1);

        size_t current_trues = 0;
        while (current_trues <=  number_of_trues)
        {
            int target = dis(gen);
            if (vec_boolean[target] == false)
            {
                vec_boolean[target] = true;
                current_trues++;
            }
        }

        NetVector vec(size);
        for (NetVector::size_type i = 0; i < size; i++)
        {
            vec[i] = vec_boolean[i] ? 1.0 : 0.0;
        }
        return vec;
    }

    inline NetVector zeros(size_t size)
    {
        NetVector vec(size);
        for (size_t i = 0; i < size; i++)
        {
            vec[i] = 0.0;
        }
        return vec;
    }

    inline NetVector ones(size_t size)
    {
        NetVector vec(size);
        for (size_t i = 0; i < size; i++)
        {
            vec[i] = 1.0;
        }
        return vec;
    }

    inline NetVector clone(const NetVector& a)
    {
        NetVector c(a.size());
        for (NetVector::size_type i = 0; i < a.size(); i++)
        {
            c[i] = a[i];
        }
        return c;
    }

    // ################################################################
    // #    vector manupulation
    // ################################################################

    inline NetVector mult(const NetVector& a, const NetVector& b)
    {
        NetVector c(a.size());
        for (NetVector::size_type i = 0; i < a.size(); i++)
        {
            c[i] = a[i] * b[i];
        }
        return c;
    }

    inline NetVector mult(const NetVector& a, fann_type scalar)
    {
        NetVector c(a.size());
        for (NetVector::size_type i = 0; i < a.size(); i++)
        {
            c[i] = a[i] * scalar;
        }
        return c;
    }

    inline NetVector add(const NetVector& a, const NetVector& b)
    {
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i] + b[i];
        }
        return c;
    }

    inline NetVector add(const NetVector& a, fann_type b)
    {
        NetVector c(a.size());
        for (NetVector::size_type i = 0; i < a.size(); i++)
        {
            c[i] = a[i] + b;
        }
        return c;
    }

    inline fann_type check_sum(const NetVector& a)
    {
        fann_type cs = 0;
        for (NetVector::size_type i = 0; i < a.size(); i++)
        {
            cs = cs + a[i];
        }
        return cs;
    }

    // ################################################################
    // #    vector mutation
    // ################################################################

    inline NetVector exchange_mutation(const NetVector& p, unsigned int switches)
    {
        NetVector::size_type size = p.size();

        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> dis(0, size - 1);

        NetVector m = clone(p);
        for (unsigned int i = 0; i < switches; i++)
        {
            int a;
            int b;
            do
            {
                a = dis(gen);
                b = dis(gen);
            } while (a != b);

            fann_type tmp = m[a];
            m[a] = m[b];
            m[b] = tmp;
        }

        return m;
    }

    inline NetVector uniform_add_mutation(const NetVector& p, double learning_rate)
    {
        NetVector random = random_normal_distribution(p.size(), 0, learning_rate);
        NetVector m = add(p, random);
        return m;
    }

    inline NetVector uniform_mult_mutation(const NetVector& p, double learning_rate)
    {
        NetVector random = random_normal_distribution(p.size(), 1, learning_rate);
        NetVector m = mult(p, random);
        return m;
    }

    // ################################################################
    // #    vector conversion
    // ################################################################

    inline std::string to_string(const NetVector& vec)
    {
        std::string str = "";
        for (NetVector::size_type i = 0; i < vec.size(); i++)
        {
            str = str + std::to_string(vec[i]) + " ";
        }
        return str;
    }

    inline NetVector net_to_vector(FANN::neural_net* net)
    {
        size_t size = net->get_total_connections();
        FANN::connection arr[size];
        net->get_connection_array(arr);
        NetVector vec(size);
        for (size_t i = 0; i < size; i++)
        {
            vec[i] = arr[i].weight;
        }
        return vec;
    }

    inline FANN::neural_net* vector_to_net(const NetVector& vec, unsigned int layers, unsigned int* layer_array)
    {
        FANN::neural_net* net = new FANN::neural_net();
        net->create_standard_array(layers, layer_array);
        size_t size = net->get_total_connections();
        FANN::connection arr[size];
        net->get_connection_array(arr);
        for (size_t i = 0; i < size; i++)
        {
            arr[i].weight = vec[i];
        }
        net->set_weight_array(arr, size);
        return net;
    }
}
