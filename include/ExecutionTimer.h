// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <chrono>
#include <iostream>
#include <ratio>
#include <vector>

namespace detail {

  template<typename clockT = std::chrono::steady_clock, class durationT = typename clockT::duration>
  class ExecutionTimer {
   public:
    ExecutionTimer() = default;
    ~ExecutionTimer() = default;
    void getClockProperties() const;
    void sample();
    void initializeMeasurement();

    [[nodiscard]] bool isInitialized() const { return _measurementInitialized; }

    /********************************************************************************************************************/

    // Measure a single execution step
    void measureOnce() {
      if(_measurementInitialized) {
        _period = clockT::now() - _previousSample;
      }
      _measurementInitialized = false;
    }

    /********************************************************************************************************************/

    // Measure multiple execution cycles and take iterative mean
    void measureIterativeMean() {
      if(_measurementInitialized) {
        _nIterations++;
        auto now = clockT::now();
        _period = _period + (now - _previousSample - _period) / _nIterations;

        _previousSample = clockT::now();
        _overhead = _previousSample - now;
      }
    }

    /********************************************************************************************************************/

    // Get current measurement result
    [[nodiscard]] durationT getMeasurementResult() const { return std::chrono::duration_cast<durationT>(_period); }

    /********************************************************************************************************************/

    // Prints measurement result to stdout
    void printMeasurementResult() const {
      std::cout << "Execution time: " << std::chrono::duration_cast<durationT>(_period).count() << std::endl;
    }

    /********************************************************************************************************************/

    // Print overhead in iterative measurement
    void printMeasurementOverhead() const {
      std::cout << "Overhead in iterative measurement was: " << std::chrono::duration_cast<durationT>(_overhead).count()
                << std::endl;
    }

   private:
    std::vector<std::chrono::microseconds> _samples{};
    typename clockT::duration _period{};
    typename clockT::duration _overhead{};
    typename clockT::time_point _previousSample{};
    durationT _measurementResult{0};
    bool _measurementInitialized{false};
    int64_t _nIterations{0};
  };
} // namespace detail

/********************************************************************************************************************/
/* Implementations                                                                                                  */
/********************************************************************************************************************/

template<typename clockT, class durationT>
void detail::ExecutionTimer<clockT, durationT>::getClockProperties() const {
  double prec = static_cast<double>(clockT::period::num) / clockT::period::den;
  std::cout << "Precision: " << prec << std::endl << "Is steady: " << std::boolalpha << clockT::is_steady << std::endl;
}

/********************************************************************************************************************/

template<typename clockT, class durationT>
void detail::ExecutionTimer<clockT, durationT>::sample() {
  _samples.push_back(clockT::now());
}

/********************************************************************************************************************/

template<typename clockT, class durationT>
void detail::ExecutionTimer<clockT, durationT>::initializeMeasurement() {
  _nIterations = 0;
  _period = durationT(0);
  _previousSample = clockT::now();
  _measurementInitialized = true;
}
