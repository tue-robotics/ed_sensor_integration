#include "ed_sensor_integration/association_matrix.h"
#include "ed_sensor_integration/hungarian.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace ed_sensor_integration
{

// ----------------------------------------------------------------------------------------------------

AssociationMatrix::AssociationMatrix(unsigned int num_measurements) : i_max_entity_(0), matrix_(num_measurements)
{
}

// ----------------------------------------------------------------------------------------------------

AssociationMatrix::~AssociationMatrix()
{
}

// ----------------------------------------------------------------------------------------------------

void AssociationMatrix::setEntry(int i_measurement, int i_entity, double prob)
{
    if (prob <= 0)
        return;

    std::vector<Entry>& msr_row = matrix_[i_measurement];
    msr_row.push_back(Entry(i_measurement, i_entity, prob));

    i_max_entity_ = std::max(i_max_entity_, i_entity);
}

// ----------------------------------------------------------------------------------------------------

bool AssociationMatrix::calculateBestAssignment(Assignment& assig)
{
    if (matrix_.empty())
    {
        assig.clear();
        return true;
    }

    int num_measurements = matrix_.size();
    int num_entities = i_max_entity_ + 1;

    // Build cost matrix for Hungarian algorithm
    // Hungarian minimizes cost, so we convert probability to cost: cost = -log(prob)
    // High probability -> low cost
    std::vector<std::vector<double>> cost_matrix(num_measurements,
                                                  std::vector<double>(num_entities, 1e9));

    for (int i = 0; i < num_measurements; ++i)
    {
        for (const Entry& entry : matrix_[i])
        {
            if (entry.i_entity >= 0)
            {
                // Convert probability to cost: higher probability = lower cost
                // Using -log(prob) as cost metric
                double cost = -std::log(entry.probability + 1e-10);
                cost_matrix[i][entry.i_entity] = cost;
            }
        }
    }

    // Solve using Hungarian algorithm
    std::vector<int> hungarian_assignment;
    double total_cost = HungarianAlgorithm::solve(cost_matrix, hungarian_assignment);

    // Convert to output format
    assig.resize(num_measurements);
    for (int i = 0; i < num_measurements; ++i)
    {
        int entity_idx = hungarian_assignment[i];

        // Check if this is a valid assignment (not dummy/unmatched)
        if (entity_idx >= 0 && entity_idx < num_entities &&
            cost_matrix[i][entity_idx] < 1e8)  // Not an impossible assignment
        {
            assig[i] = entity_idx;
        }
        else
        {
            assig[i] = -1;  // No assignment
        }
    }

    return true;
}

}
