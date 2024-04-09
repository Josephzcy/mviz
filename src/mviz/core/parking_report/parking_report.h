// Copyright 2023 MINIEYE

#ifndef PARKING_REPORT_H_
#define PARKING_REPORT_H_
#include <iostream>

#include "json.hpp"
namespace parking_report {
class ParkingReport final {
 public:
  explicit ParkingReport(const std::string& mviz_dir) noexcept;
  ~ParkingReport() noexcept{};
  explicit ParkingReport(nlohmann::json& parking_report_json) noexcept;

  ParkingReport(const ParkingReport&) = delete;
  ParkingReport(ParkingReport&&) = delete;
  ParkingReport& operator=(const ParkingReport&) = delete;
  ParkingReport& operator=(ParkingReport&&) = delete;

 public:
  nlohmann::json parking_report_json_;

 private:
  const std::string report_path_;
};

#endif
}
