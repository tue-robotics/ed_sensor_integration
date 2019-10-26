#ifndef ED_SENSOR_INTEGRATION_ASSOCIATION_MATRIX_H_
#define ED_SENSOR_INTEGRATION_ASSOCIATION_MATRIX_H_

#include <vector>

namespace ed_sensor_integration
{

typedef std::vector<int> Assignment;


class AssociationMatrix
{

public:

    struct Entry
    {
        Entry(int i_msr_, int i_entity_, double prob_)
            : i_measurement(i_msr_), i_entity(i_entity_), probability(prob_) {}

        int i_measurement;
        int i_entity;
        double probability;
    };

    AssociationMatrix(unsigned int num_measurements);

    ~AssociationMatrix();

    void setEntry(int i_measurement, int i_entity, double prob);

    bool calculateBestAssignment(Assignment& assig);

//    static int NO_ASSIGNMENT;

private:

    int i_max_entity_;

    std::vector<std::vector<Entry> > matrix_;
};

}

#endif
