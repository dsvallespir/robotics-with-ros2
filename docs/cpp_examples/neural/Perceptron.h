#include <vector.h>

class Perceptron {
    public:
        double run (std::vector<double> data_input);

        void set_weights(std::vector<double> initial_weights);
        double activation_function(double x);
        double bias;
        
}