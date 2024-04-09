#include "ParkingReport.h"
namespace parking_report {
ParkingReport::ParkingReport(const std::string& mviz_dir) : report_path_(mviz_dir+"/parking_report.json") {
  ParkingReport(parking_report_json_);
}

ParkingReport::ParkingReport(nlohmann::json& parking_report_json) {

  parking_report_json["scene"] = "泛悦城车库";
  parking_report_json["slot_type"] = "vertical";
  parking_report_json["function"] = "in_parking";
  parking_report_json["version"] = "apa_v2.0.4";
  
  json vehicle_info;   // 新的字段
  vehicle_info["make"] = "Tesla";
  vehicle_info["model"] = "Model 3";
  vehicle_info["year"] = 2023;
  vehicle_info["license_plate"] = "ABC123";

  parking_report_json["vehicle_info"] = vehicle_info;

  json kpi;   // 新的字段，用时和换挡次数
  kpi.push_back("kpi");

  // std::string json_str = document.dump();
  // std::cout << json_str << std::endl;

  parking_report_json["kpi"] = kpi;
}

}  // namespace parking_report
