#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace acs::testing {

class TestResult {
public:
  TestResult(const std::string& name) : test_name_(name) {}

  void pass() {
    passed_ = true;
    message_ = "PASS";
  }

  void fail(const std::string& message) {
    passed_ = false;
    message_ = "FAIL: " + message;
  }

  bool passed() const { return passed_; }
  const std::string& name() const { return test_name_; }
  const std::string& message() const { return message_; }

private:
  std::string test_name_;
  bool passed_{false};
  std::string message_;
};

class TestSuite {
public:
  using TestFunc = std::function<void(TestResult&)>;

  TestSuite(const std::string& name) : suite_name_(name) {}

  void add_test(const std::string& name, TestFunc func) {
    tests_.push_back({name, func});
  }

  void run() {
    std::cout << "\n=== Running Test Suite: " << suite_name_ << " ===\n";

    for (const auto& [name, func] : tests_) {
      TestResult result(name);
      try {
        func(result);
      } catch (const std::exception& e) {
        result.fail(std::string("Exception: ") + e.what());
      } catch (...) {
        result.fail("Unknown exception");
      }

      results_.push_back(result);

      std::cout << "  [" << (result.passed() ? " OK " : "FAIL") << "] "
                << result.name();
      if (!result.passed()) {
        std::cout << " - " << result.message();
      }
      std::cout << "\n";
    }

    print_summary();
  }

  size_t num_passed() const {
    size_t count = 0;
    for (const auto& result : results_) {
      if (result.passed()) count++;
    }
    return count;
  }

  size_t num_failed() const {
    return results_.size() - num_passed();
  }

private:
  void print_summary() {
    std::cout << "\n--- Summary ---\n";
    std::cout << "Total:  " << results_.size() << "\n";
    std::cout << "Passed: " << num_passed() << "\n";
    std::cout << "Failed: " << num_failed() << "\n";
    std::cout << "===============\n\n";
  }

  std::string suite_name_;
  std::vector<std::pair<std::string, TestFunc>> tests_;
  std::vector<TestResult> results_;
};

// Assertion helpers
template <typename T>
void assert_equal(TestResult& result, const T& expected, const T& actual,
                  const std::string& message = "") {
  if (expected == actual) {
    result.pass();
  } else {
    result.fail(message.empty() ? "Values not equal" : message);
  }
}

inline void assert_near(TestResult& result, double expected, double actual,
                       double tolerance = 1e-6, const std::string& message = "") {
  if (std::abs(expected - actual) <= tolerance) {
    result.pass();
  } else {
    result.fail(message.empty() ? "Values not near" : message);
  }
}

inline void assert_true(TestResult& result, bool condition,
                       const std::string& message = "") {
  if (condition) {
    result.pass();
  } else {
    result.fail(message.empty() ? "Condition is false" : message);
  }
}

inline void assert_false(TestResult& result, bool condition,
                        const std::string& message = "") {
  if (!condition) {
    result.pass();
  } else {
    result.fail(message.empty() ? "Condition is true" : message);
  }
}

}  // namespace acs::testing
