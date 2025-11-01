#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace jj
{
    /*
     *  C++20 introduced a new feature called concepts and contraints, which in theory should
     *  make generics more stable, by letting us tag them with contraints.
     *  Let us try to use them
     */
    // template <class T>
    // concept Container = requires (T c)
    //                     {
    //                         c.begin();
    //                         c.end();
    //                     };

    /*
     *  These are helper functions to make my life easier
     */

    template <typename C> auto max(const C &container)
    {
        auto max = *container.begin();
        for (auto p = container.begin() + 1; p != container.end(); ++p)
        {
            if (*p > max)
            {
                max = *p;
            }
        }
        return max;
    }

    template <typename C> auto min(const C &container)
    {
        auto min = *container.begin();
        for (auto p = container.begin() + 1; p != container.end(); ++p)
        {
            if (*p < min)
            {
                min = *p;
            }
        }
        return min;
    }

    template <typename C> bool is_sorted(const C &container)
    {
        for (auto p = container.begin(); p != container.end() - 1; ++p)
        {
            if (!(*p <= *(p + 1)))
            {
                return false;
            }
        }
        return true;
    }

    template <typename C, typename S> void print_container(const C &container, S &stream)
    {
        for (auto p = container.begin(); p != container.end(); ++p)
        {
            stream << *p << " ";
        }
        stream << "\n";
    }
    template <typename I, typename S> void print_container(I begin, I end, S &stream)
    {
        for (auto p = begin; p != end; ++p)
        {
            stream << *p << " ";
        }
        stream << "\n";
    }

    /*
     *  These are impl of different sorting algo
     */

    // Counting Sort
    template <typename C> void counting_sort(C &container)
    {
        size_t keys_length = max<C>(container) + 1;
        size_t *keys = new size_t[keys_length]();

        for (auto p = container.end() - 1; p != container.begin() - 1; --p)
        {
            ++keys[*p];
        }

        for (size_t i = 1; i < keys_length; ++i)
        {
            keys[i] = keys[i] + keys[i - 1];
        }

        C sorted(container.size());

        for (auto p = container.end() - 1; p != container.begin() - 1; --p)
        {
            sorted[keys[*p] - 1] = *p;
            --keys[*p];
        }

        delete[] keys;
        container = std::move(sorted);
    }

    // Counting Sort
    template <typename C> void counting_sort_binary(C &container, unsigned int shift)
    {
        size_t *keys = new size_t[2]();

        for (auto p = container.end() - 1; p != container.begin() - 1; --p)
        {
            ++keys[(*p >> shift) & 0b1];
        }

        for (size_t i = 1; i < 2; ++i)
        {
            keys[i] = keys[i] + keys[i - 1];
        }

        C sorted(container.size());

        for (auto p = container.end() - 1; p != container.begin() - 1; --p)
        {
            sorted[keys[(*p >> shift) & 0b1]-- - 1] = *p;
        }

        delete[] keys;
        container = std::move(sorted);
    }

    // Radix Sort ; using binary?
    template <typename C> void radix_sort(C &container)
    {
        for (size_t p = 0; p < sizeof(container[0]) * 8; ++p)
        {
            counting_sort_binary(container, p);
        }
    }

    // Bubble Sort
    template <typename C> void bubble_sort(C &container)
    {
        for (auto i = container.end(); i != container.begin(); --i)
        {
            for (auto p = container.begin(); p != i - 1; ++p)
            {
                if (!(*p <= *(p + 1)))
                {
                    std::swap(*p, *(p + 1));
                }
            }
        }
    }

    template <typename C> void insertion_sort(C &container)
    {
        for (auto i = container.begin() + 1; i != container.end(); ++i)
        {
            for (auto j = i; j != (container.begin() + 1); --j)
            {
                if (!(*(j - 1) <= *(j)))
                {
                    std::swap(*(j - 1), *j);
                }
                else
                {
                    break;
                }
            }
        }
    }

    template <typename C> void selection_sort(C &container)
    {
        for (auto i = container.begin(); i != container.end(); ++i)
        {
            auto min = i;
            for (auto j = i + 1; j != container.end(); ++j)
            {
                if (*j <= *min)
                {
                    min = j;
                }
            }
            std::swap(*i, *min);
        }
    }

    /*
     * These are different algorithms that we went over in Design and Analysis of Algorithms.
     * I wrote these to better understand the topics.
     */

    static std::map<uint64_t, uint64_t> fibonacci_memory({{0, 0}, {1, 1}});
    uint64_t fibonacci_dynamic(uint64_t n)
    {
        if (n > 1 && fibonacci_memory.find(n) == fibonacci_memory.end())
        {
            fibonacci_memory.insert_or_assign(n, fibonacci_dynamic(n - 1) + fibonacci_dynamic(n - 2));
        }
        return fibonacci_memory.at(n);
    }

    using complex = std::complex<long double>;
    void FFT(std::vector<complex> &coefficients)
    {
        while (coefficients.size() != std::bit_ceil(coefficients.size()))
        {
            coefficients.push_back(0);
        }

        const size_t half_size = coefficients.size() / 2;
        std::vector<complex> even(half_size), odd(half_size);

        for (size_t i = 0; i < half_size; ++i)
        {
            even[i] = coefficients[i * 2];
            odd[i] = coefficients[i * 2 + 1];
        }

        if (half_size > 1)
        {
            FFT(even);
            FFT(odd);
        }

        for (size_t k = 0; k < half_size; ++k)
        {
            complex C_W = std::exp(-M_PIl * complex(0, 1) / (long double)half_size * (long double)k) * odd[k];
            coefficients[k] = even[k] + C_W;
            coefficients[k + half_size] = even[k] - C_W;
        }
    }

    std::vector<uint32_t> longest_subsequence(std::vector<uint32_t> sequence)
    {
        // Create the "cache" for our DP problem
        std::vector<std::vector<uint32_t>> cache(sequence.size());

        // Initialize all the vectors O(n)
        for (std::size_t i = 0; i < sequence.size(); ++i)
        {
            cache.at(i) = std::vector<uint32_t>();
            cache.at(i).push_back(sequence.at(i));
        }

        // Do the search, O(n^2)
        for (std::size_t i = 0; i < sequence.size(); ++i)
        {
            for (std::size_t j = i + 1; j < sequence.size(); ++j)
            {
                if (sequence.at(j) > cache.at(i).back())
                {
                    cache.at(i).push_back(sequence.at(j));
                }
            }
        }

        // Extract the max, O(n)
        std::size_t largest_index = 0;
        std::size_t largest_value = cache.at(0).size();
        for (std::size_t i = 1; i < cache.size(); ++i)
        {
            if (cache.at(i).size() > largest_value)
            {
                largest_index = i;
                largest_value = cache.at(i).size();
            }
        }

        return cache.at(largest_index);
    }

    class Graph
    {
      public:
        using Vertex = std::size_t;
        using Weight = double;
        struct Edge
        {
            Vertex u;
            Vertex v;
            Weight w;
        };
        using Adj_List = std::unordered_map<Vertex, Weight>;

        std::unordered_map<Vertex, Adj_List> graph;
        std::size_t num_of_vertex = 0;
        std::size_t num_of_edge = 0;

        void load_undirected_weighted(std::string file)
        {
            std::ifstream in(file);
            if (!in)
            {
                throw std::runtime_error("Cannot open file");
            }

            Edge e;
            while (in >> e.u >> e.v >> e.w)
            {
                ++num_of_edge;
                graph[e.u - 1][e.v - 1] = e.w;
                graph[e.v - 1][e.u - 1] = e.w;
            }

            num_of_vertex = graph.size();
        }

        using pq_type = std::pair<Vertex, Weight>;
        constexpr static auto pq_comp = [](const pq_type &left, const pq_type &right) {
            return left.second > right.second;
        };

        // Make sure to zero index
        std::vector<pq_type> Dijkstra(Vertex start)
        {
            std::vector<pq_type> prev_dist(num_of_vertex, {-1, 1.0e12});
            std::priority_queue<pq_type, std::vector<pq_type>, decltype(pq_comp)> queue(pq_comp);

            queue.push({start, 0.0});
            prev_dist[start] = {-1, 0.0};

            while (!queue.empty())
            {
                auto [u, u_w] = queue.top();
                queue.pop();

                for (const auto &[v, v_w] : graph[u])
                {
                    auto alt = prev_dist[u].second + v_w;
                    if (alt < prev_dist[v].second)
                    {
                        prev_dist[v] = {u, alt};
                        queue.push({v, alt});
                    }
                }
            }

            return prev_dist;
        }
    };

    /*
        score_t: the type that the score will be eg float etc
        node_t: the type that each node of the graph will be, eg game state
        index_t: the type that will be the index type, usually std::size_t
        is_terminal: function that checks if game is done
        heuristic: function that returns a score for the node
        subsequent: function that generates subsequent nodes to check (this is children)
     */
    template <typename score_t, typename node_t, typename index_t, auto is_terminal, auto heuristic, auto subsequent>
    score_t alphabeta(const node_t &node, const index_t &depth, score_t alpha, score_t beta,
                      const bool &maximizing_player)
    {
        if (depth == 0 || is_terminal(node))
        {
            return heuristic(node);
        }

        std::vector<node_t> children = subsequent(node);

        if (maximizing_player)
        {
            score_t value = std::numeric_limits<score_t>::min() / 2;
            for (const node_t &child : children)
            {
                value = std::max(value, alphabeta<score_t, node_t, index_t, is_terminal, heuristic, subsequent>(
                                            child, depth - 1, alpha, beta, false));
                if (value >= beta)
                {
                    break;
                }
                alpha = std::max(alpha, value);
            }
            return value;
        }
        else
        {
            score_t value = std::numeric_limits<score_t>::max() / 2;
            for (const node_t &child : children)
            {
                value = std::min(value, alphabeta<score_t, node_t, index_t, is_terminal, heuristic, subsequent>(
                                            child, depth - 1, alpha, beta, true));
                if (value <= alpha)
                {
                    break;
                }
                beta = std::min(beta, value);
            }
            return value;
        }
    }

} // namespace jj
