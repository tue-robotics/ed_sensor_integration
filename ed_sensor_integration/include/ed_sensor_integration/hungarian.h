#ifndef ED_SENSOR_INTEGRATION_HUNGARIAN_H_
#define ED_SENSOR_INTEGRATION_HUNGARIAN_H_

#include <vector>
#include <limits>

namespace ed_sensor_integration
{

/**
 * @brief Hungarian algorithm (Kuhn-Munkres) for optimal assignment problem
 *
 * Solves the linear assignment problem in O(n³) time.
 * Given a cost matrix, finds the assignment that minimizes total cost.
 * Handles rectangular matrices (more rows or columns).
 */
class HungarianAlgorithm
{
public:
    /**
     * @brief Solve the assignment problem
     *
     * @param cost_matrix Cost matrix where cost_matrix[i][j] is the cost of assigning row i to column j.
     *                    Lower cost = better match. Use large value (e.g., 1e9) for impossible assignments.
     * @param assignment Output vector where assignment[i] = j means row i is assigned to column j.
     *                   assignment[i] = -1 means row i is unassigned.
     * @return double Total cost of the optimal assignment
     */
    static double solve(const std::vector<std::vector<double>>& cost_matrix,
                       std::vector<int>& assignment);

private:
    static constexpr double INF = std::numeric_limits<double>::max() / 2.0;

    // Internal solve method that works on a square matrix
    static double solveSquare(std::vector<std::vector<double>>& cost_matrix,
                             std::vector<int>& assignment,
                             int n, int m);

    // Helper methods for the Hungarian algorithm
    static void subtractRowMinima(std::vector<std::vector<double>>& cost, int n, int m);
    static void subtractColMinima(std::vector<std::vector<double>>& cost, int n, int m);
    static bool findAugmentingPath(const std::vector<std::vector<double>>& cost,
                                   std::vector<int>& row_mate,
                                   std::vector<int>& col_mate,
                                   std::vector<double>& row_slack,
                                   std::vector<double>& col_slack,
                                   std::vector<int>& parent_row,
                                   std::vector<bool>& row_visited,
                                   std::vector<bool>& col_visited,
                                   int n, int m, int unmatched_row);
};

} // namespace ed_sensor_integration

#endif // ED_SENSOR_INTEGRATION_HUNGARIAN_H_
