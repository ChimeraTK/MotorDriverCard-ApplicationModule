#ifndef EXECUTION_TIMER_H
#define EXECUTION_TIMER_H

#include <iostream>
#include <chrono>
#include <vector>
#include <ratio>


template <typename clockT = std::chrono::steady_clock, class durationT = typename clockT::duration>
class ExecutionTimer {

    std::vector<std::chrono::microseconds> samples;
    typename clockT::duration period, overhead;
    typename clockT::time_point previousSample;
    durationT measurement_result;
    bool measurementInitialized = false;
    int64_t nIterations;

  public:
    ExecutionTimer(){}
    ~ExecutionTimer(){}
    void getClockProperties(void);
    void sample(void);
    void initializeMeasurement(void);

    bool isInitialized(void){
      return measurementInitialized;
    }

    // Measure a single execution step 
    void measureOnce(void){
        if(measurementInitialized){
            period = clockT::now() - previousSample;    
        }
        measurementInitialized = false;
    };

    // Measure multiple execution cycles and take iterative mean
    void measureIterativeMean(void){
        if(measurementInitialized){
            nIterations++;
            auto now = clockT::now();
            period = period + (now - previousSample - period)/nIterations;

            previousSample = clockT::now();
            overhead = previousSample - now;
        }
    };

    // Get current measurement result
    durationT getMeasurementResult(){
        return std::chrono::duration_cast<durationT>(period);
    };

    // Prints measurement result to stdout
    void printMeasurementResult(){
        std::cout << "Execution time: " << std::chrono::duration_cast<durationT>(period).count() 
                  << std::endl;
    };

    // Print overhead in iterative measurement
    void printMeasurementOverhead(void){
        std::cout << "Overhead in iterative measurement was: " 
                  << std::chrono::duration_cast<durationT>(overhead).count()
                  << std::endl;
    };
};



template <typename clockT, class durationT>
void ExecutionTimer<clockT, durationT>::getClockProperties(){

    double prec = static_cast<double>(clockT::period::num)/clockT::period::den;
    std::cout << "Precision: " << prec << std::endl
              << "Is steady: " << std::boolalpha << clockT::is_steady << std::endl;
}

template <typename clockT, class durationT>
void ExecutionTimer<clockT, durationT>::sample(){
    samples.push_back(clockT::now());
}

template <typename clockT, class durationT>
void ExecutionTimer<clockT, durationT>::initializeMeasurement(void){

    nIterations = 0;
    period = durationT(0);
    previousSample = clockT::now();
    measurementInitialized = true;
}


#endif // EXECUTION_TIMER_H 
