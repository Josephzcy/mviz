//
// Created by nick on 23-3-28.
//

#ifndef MVIZ_RECORD_FILESYSTEM_HPP_
#define MVIZ_RECORD_FILESYSTEM_HPP_
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <iostream>

#include "glog/logging.h"
#include "json.hpp"
namespace minieye::mviz::file {

class CFileSys {
 public:
  explicit CFileSys(const char *saveBatchIdRootDir) { str_cur_batch_id_root_dir = saveBatchIdRootDir; }
  ~CFileSys() = default;

  int file_sys_initialize(const std::string &topic_name) {
    if (create_file_struct(topic_name)) return 0;
    return -1;
  }

  void init_write_index_json_head(nlohmann::json &totalIndexJson) {
    totalIndexJson["key"] = "timestamp";
    totalIndexJson["fields"]["timestamp"] = 0;
    totalIndexJson["fields"]["offset"] = 1;
    totalIndexJson["fields"]["size"] = 2;
    totalIndexJson["fields"]["frame_idx"] = 3;
    totalIndexJson["fields"]["tick"] = 4;
    totalIndexJson["index"] = {};
  }

  void init_write_img_index_json_head(nlohmann::json &totalIndexJson) {
    totalIndexJson["key"] = "timestamp";
    totalIndexJson["fields"]["timestamp"] = 0;
    totalIndexJson["fields"]["offset"] = 1;
    totalIndexJson["fields"]["size"] = 2;
    totalIndexJson["fields"]["frame_id"] = 3;
    totalIndexJson["fields"]["frame_idx"] = 4;
    totalIndexJson["fields"]["video_idx"] = 5;
    totalIndexJson["fields"]["camera_id"] = 6;
    totalIndexJson["fields"]["frame_type"] = 7;
    totalIndexJson["fields"]["tick"] = 8;
    totalIndexJson["index"] = {};
  }

  void file_sys_finalize() { release_files(); }

  virtual int write_bin_data(const char *buff, size_t n) = 0;
  //{
  // if (binFileOFS.is_open()){
  // binFilePath << buff;
  //}
  //};

  virtual int write_data_json(bool isImg, int32_t height = 0, int32_t width = 0) = 0;

  virtual int write_index_json(const nlohmann::json &one_frame_info) = 0;

  virtual void save_to_json(const nlohmann::json &minieye_obj, const std::string &dst_dir,
                            const std::string &file_path) = 0;

 protected:
  void release_files() {
    binFileOFS.close();
    dataJsonFileOFS.close();
    indexJsonFileOFS.close();
    LOG(INFO) << "file_save: [ " << binFilePath.string().c_str() << "]  close file is ok..";
  }

  bool create_file_struct(const std::string &topic_name) {
    boost::filesystem::path path(str_cur_batch_id_root_dir.c_str());
    batchDir = path;
    
    std::string _topic_name=("camera30" == topic_name ? "camera_stitching" : topic_name);
    std::string binPathName = _topic_name + ".bin";
    std::string dataJsonPathName = _topic_name + ".data.json";
    std::string indexJsonPathName = _topic_name + ".index.json";

    if (!boost::filesystem::exists(batchDir)) return false;

    binFilePath = batchDir / binPathName.c_str();
    binFileOFS.open(binFilePath, std::ios_base::out | std::ios_base::app | std::ios_base::binary);
    LOG(INFO) << "file_save: [ " << binFilePath.string().c_str() << "]  open file is ok..";

    // create json files
    dataJsonPath = batchDir / dataJsonPathName.c_str();
    indexJsonPath = batchDir / indexJsonPathName.c_str();
    dataJsonFileOFS.open(dataJsonPath, std::ios_base::out | std::ios_base::app | std::ios_base::binary);
    indexJsonFileOFS.open(indexJsonPath, std::ios_base::out | std::ios_base::app | std::ios_base::binary);

    //      dataJsonFileOFS.open(dataJsonPath, std::ios_base::out |
    //      std::ios_base::app | std::ios_base::binary); LOG(INFO) <<
    //      "file_save: [ " << dataJsonPath.string().c_str() << "]  open file
    //      is ok.."; indexJsonFileOFS.open(indexJsonPath, std::ios_base::out
    //      | std::ios_base::app | std::ios_base::binary); LOG(INFO) <<
    //      "file_save: [ " << indexJsonPath.string().c_str() << "]  open file
    //      is ok..";

    return true;
  };

  std::string str_cur_batch_id_root_dir;
  boost::filesystem::path batchDir;
  boost::filesystem::path binFilePath;
  boost::filesystem::path dataJsonPath;
  boost::filesystem::path indexJsonPath;
  boost::filesystem::fstream binFileOFS;
  boost::filesystem::fstream dataJsonFileOFS;
  boost::filesystem::fstream indexJsonFileOFS;
};

}  // namespace minieye::mviz::file

#endif  // MVIZ_RECORD_FILESYSTEM_HPP_
