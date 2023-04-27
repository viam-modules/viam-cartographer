#include "slam_service.h"

#include <boost/test/unit_test.hpp>
#include <exception>

#include "../io/file_handler.h"
#include "../utils/slam_service_helpers.h"
#include "../utils/test_helpers.h"

namespace viam {
namespace {

BOOST_AUTO_TEST_SUITE(SLAMService)

void checkCartoMapBuilderParameters(SLAMServiceImpl& slamService) {
    auto tolerance = boost::test_tools::tolerance(0.00001);

    BOOST_TEST(slamService.GetOptimizeEveryNNodesFromMapBuilder() ==
               slamService.optimize_every_n_nodes);
    BOOST_TEST(slamService.GetNumRangeDataFromMapBuilder() ==
               slamService.num_range_data);
    BOOST_TEST(slamService.GetMissingDataRayLengthFromMapBuilder() ==
                   slamService.missing_data_ray_length,
               tolerance);
    BOOST_TEST(slamService.GetMaxRangeFromMapBuilder() == slamService.max_range,
               tolerance);
    BOOST_TEST(slamService.GetMinRangeFromMapBuilder() == slamService.min_range,
               tolerance);
    if (slamService.GetActionMode() == ActionMode::LOCALIZING) {
        BOOST_TEST(slamService.GetMaxSubmapsToKeepFromMapBuilder() ==
                   slamService.max_submaps_to_keep);
    } else {
        BOOST_TEST(slamService.GetMaxSubmapsToKeepFromMapBuilder() == 0);
    }

    if (slamService.GetActionMode() == ActionMode::UPDATING) {
        BOOST_TEST(slamService.GetFreshSubmapsCountFromMapBuilder() ==
                   slamService.fresh_submaps_count);
        BOOST_TEST(slamService.GetMinCoveredAreaFromMapBuilder() ==
                       slamService.min_covered_area,
                   tolerance);
        BOOST_TEST(slamService.GetMinAddedSubmapsCountFromMapBuilder() ==
                   slamService.min_added_submaps_count);
    } else {
        BOOST_TEST(slamService.GetFreshSubmapsCountFromMapBuilder() == 0);
        BOOST_TEST(slamService.GetMinCoveredAreaFromMapBuilder() == 0,
                   tolerance);
        BOOST_TEST(slamService.GetMinAddedSubmapsCountFromMapBuilder() == 0);
    }
    BOOST_TEST(slamService.GetOccupiedSpaceWeightFromMapBuilder() ==
                   slamService.occupied_space_weight,
               tolerance);
    BOOST_TEST(slamService.GetTranslationWeightFromMapBuilder() ==
                   slamService.translation_weight,
               tolerance);
    BOOST_TEST(slamService.GetRotationWeightFromMapBuilder() ==
                   slamService.rotation_weight,
               tolerance);
}

BOOST_AUTO_TEST_CASE(OverwriteMapBuilderParameters_set_values_mapping) {
    SLAMServiceImpl slamService;
    slamService.optimize_every_n_nodes = 9999;
    slamService.num_range_data = 9998;
    slamService.missing_data_ray_length = 9997.77;
    slamService.max_range = 9996.66;
    slamService.min_range = 9995.55;
    slamService.max_submaps_to_keep = 9994;
    slamService.fresh_submaps_count = 9993;
    slamService.min_covered_area = 9992.22;
    slamService.min_added_submaps_count = 9991;
    slamService.occupied_space_weight = 9990.09;
    slamService.translation_weight = 9989.89;
    slamService.rotation_weight = 9988.88;

    // Mapping is the default action_mode when slamService is created
    BOOST_TEST(slamService.GetActionMode() == ActionMode::MAPPING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);
}

BOOST_AUTO_TEST_CASE(
    OverwriteMapBuilderParameters_check_default_values_mapping) {
    SLAMServiceImpl slamService;

    // Mapping is the default action_mode when slamService is created
    BOOST_TEST(slamService.GetActionMode() == ActionMode::MAPPING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);
}

BOOST_AUTO_TEST_CASE(OverwriteMapBuilderParameters_set_values_updating) {
    // Create a temp directory with a map file in it
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{
        "map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);

    SLAMServiceImpl slamService;
    slamService.path_to_map = tmp_dir.string() + "/map";
    slamService.optimize_every_n_nodes = 9999;
    slamService.num_range_data = 9998;
    slamService.missing_data_ray_length = 9997.77;
    slamService.max_range = 9996.66;
    slamService.min_range = 9995.55;
    slamService.max_submaps_to_keep = 9994;
    slamService.fresh_submaps_count = 9993;
    slamService.min_covered_area = 9992.22;
    slamService.min_added_submaps_count = 9991;
    slamService.occupied_space_weight = 9990.09;
    slamService.translation_weight = 9989.89;
    slamService.rotation_weight = 9988.88;
    // Set the action mode to updating by providing an apriori map
    // and by setting map_rate_sec != 0
    slamService.map_rate_sec = std::chrono::seconds(60);

    slamService.SetActionMode();

    BOOST_TEST(slamService.GetActionMode() == ActionMode::UPDATING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_CASE(
    OverwriteMapBuilderParameters_check_default_values_updating) {
    // Create a temp directory with a map file in it
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{
        "map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);

    SLAMServiceImpl slamService;
    slamService.path_to_map = tmp_dir.string() + "/map";
    // Set the action mode to updating by providing an apriori map
    // and by setting map_rate_sec != 0
    slamService.map_rate_sec = std::chrono::seconds(60);

    slamService.SetActionMode();

    BOOST_TEST(slamService.GetActionMode() == ActionMode::UPDATING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_CASE(OverwriteMapBuilderParameters_set_values_localizing) {
    // Create a temp directory with a map file in it
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{
        "map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);

    SLAMServiceImpl slamService;
    slamService.path_to_map = tmp_dir.string() + "/map";
    slamService.optimize_every_n_nodes = 9999;
    slamService.num_range_data = 9998;
    slamService.missing_data_ray_length = 9997.77;
    slamService.max_range = 9996.66;
    slamService.min_range = 9995.55;
    slamService.max_submaps_to_keep = 9994;
    slamService.fresh_submaps_count = 9993;
    slamService.min_covered_area = 9992.22;
    slamService.min_added_submaps_count = 9991;
    slamService.occupied_space_weight = 9990.09;
    slamService.translation_weight = 9989.89;
    slamService.rotation_weight = 9988.88;
    // Set the action mode to localizing by providing an apriori map
    // and by setting map_rate_sec == 0
    slamService.map_rate_sec = std::chrono::seconds(0);

    slamService.SetActionMode();

    BOOST_TEST(slamService.GetActionMode() == ActionMode::LOCALIZING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_CASE(
    OverwriteMapBuilderParameters_check_default_values_localizing) {
    // Create a temp directory with a map file in it
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{
        "map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);

    SLAMServiceImpl slamService;
    slamService.path_to_map = tmp_dir.string() + "/map";
    // Set the action mode to localizing by providing an apriori map
    // and by setting map_rate_sec == 0
    slamService.map_rate_sec = std::chrono::seconds(0);

    slamService.SetActionMode();

    BOOST_TEST(slamService.GetActionMode() == ActionMode::LOCALIZING);
    slamService.OverwriteMapBuilderParameters();
    checkCartoMapBuilderParameters(slamService);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_CASE(SetActionMode_mapping) {
    SLAMServiceImpl slamService;
    // Mapping is the default action_mode when slamService is created
    BOOST_TEST(slamService.GetActionMode() == ActionMode::MAPPING);

    // Set up the environment such that SetActionMode sets the
    // action_mode to "mapping" by setting map_rate_sec != 0 and by
    // ensuring that there is no map in the map directory
    slamService.map_rate_sec = std::chrono::seconds(60);

    // Create a temp directory that does not contain a map
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmp_dir.string() + "/map";
    slamService.SetActionMode();

    BOOST_TEST(slamService.GetActionMode() == ActionMode::MAPPING);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_CASE(SetActionMode_updating) {
    SLAMServiceImpl slamService;

    // Set up the environment such that SetActionMode sets the
    // action_mode to "updating" by setting map_rate_sec != 0 and by
    // ensuring that there is a map in the map directory
    slamService.map_rate_sec = std::chrono::seconds(60);

    // Create a temp directory with a map file in it
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{
        "map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmp_dir.string() + "/map";
    slamService.SetActionMode();

    BOOST_TEST(slamService.GetActionMode() == ActionMode::UPDATING);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_CASE(SetActionMode_localizing) {
    SLAMServiceImpl slamService;

    // Set up the environment such that SetActionMode sets the
    // action_mode to "localizing" by setting map_rate_sec == 0 and by
    // ensuring that there is a map in the map directory
    slamService.map_rate_sec = std::chrono::seconds(0);

    // Create a temp directory with a map file in it
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{
        "map_data_2022-02-11T01:44:53.1903Z.pbstream"};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmp_dir.string() + "/map";
    slamService.SetActionMode();

    BOOST_TEST(slamService.GetActionMode() == ActionMode::LOCALIZING);

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_CASE(SetActionMode_invalid_case) {
    SLAMServiceImpl slamService;

    // Set up the environment such that SetActionMode throws
    // an error indicating that this is an invalid case. Do this by
    // setting map_rate_sec == 0 and by ensuring that there is no
    // map in the map directory
    slamService.map_rate_sec = std::chrono::seconds(0);

    // Create a temp directory that does not contain a map
    std::vector<std::string> data_files{};
    std::vector<std::string> map_files{};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_map = tmp_dir.string() + "/map";

    const std::string message =
        "set to localization mode (map_rate_sec = 0) but couldn't find "
        "apriori map to localize on";
    BOOST_CHECK_EXCEPTION(slamService.SetActionMode(), std::runtime_error,
                          [&message](const std::runtime_error& ex) {
                              BOOST_CHECK_EQUAL(ex.what(), message);
                              return true;
                          });

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

// BOOST_AUTO_TEST_CASE(GetNextDataFileOffline_not_enough_data) {
//     SLAMServiceImpl slamService;

//     // Create a temp directory that does not contain enough data files for
//     // mapping
//     std::vector<std::string> data_files{
//         "rplidar_data_2022-02-11T01:45:47.0764Z.pcd",
//         "rplidar_data_2022-02-11T01:46:41.4989Z.pcd"};
//     std::vector<std::string> map_files{};
//     // Create a unique path in the temp directory and add the files
//     boost::filesystem::path tmp_dir =
//         utils::createTmpDirectoryAndAddFiles(data_files, map_files);
//     slamService.path_to_data = tmp_dir.string() + "/data";

//     const std::string message = "not enough data in data directory";
//     BOOST_CHECK_EXCEPTION(slamService.GetNextDataFileOffline(),
//                           std::runtime_error,
//                           [&message](const std::runtime_error& ex) {
//                               BOOST_CHECK_EQUAL(ex.what(), message);
//                               return true;
//                           });

//     // Remove the temporary directory and its contents
//     utils::removeTmpDirectory(tmp_dir);
// }

// BOOST_AUTO_TEST_CASE(GetNextDataFileOffline) {
//     SLAMServiceImpl slamService;

//     // Create a temp directory with some data files in it
//     std::vector<std::string> data_files{
//         "rplidar_data_2022-02-11T01:45:47.0764Z.pcd",
//         "rplidar_data_2022-02-11T01:45:47.2439Z.pcd",
//         "rplidar_data_2022-02-11T01:46:41.4989Z.pcd",
//         "rplidar_data_2022-02-11T01:46:41.5808Z.pcd",
//         "rplidar_data_2022-02-11T01:46:41.6631Z.pcd"};
//     std::vector<std::string> map_files{};
//     // Create a unique path in the temp directory and add the files
//     boost::filesystem::path tmp_dir =
//         utils::createTmpDirectoryAndAddFiles(data_files, map_files);
//     slamService.path_to_data = tmp_dir.string() + "/data";

//     for (int i = 0; i < data_files.size(); i++) {
//         BOOST_TEST(slamService.GetNextDataFileOffline() ==
//                    (slamService.path_to_data + "/" + data_files[i]));
//     }

//     // Remove the temporary directory and its contents
//     utils::removeTmpDirectory(tmp_dir);
// }

BOOST_AUTO_TEST_CASE(GetNextDataFileOnline) {
    SLAMServiceImpl slamService;

    // Create a temp directory with some data files in it
    std::vector<std::string> data_files{
        "rplidar_data_2022-02-11T01:45:47.0764Z.pcd",
        "rplidar_data_2022-02-11T01:45:47.2439Z.pcd",
        "rplidar_data_2022-02-11T01:46:41.4989Z.pcd",
        "rplidar_data_2022-02-11T01:46:41.5808Z.pcd",
        "rplidar_data_2022-02-11T01:46:41.6631Z.pcd"};
    std::vector<std::string> map_files{};
    // Create a unique path in the temp directory and add the files
    boost::filesystem::path tmp_dir =
        utils::createTmpDirectoryAndAddFiles(data_files, map_files);
    slamService.path_to_data = tmp_dir.string() + "/data";

    for (int i = data_files.size() - 1; i > 0; i--) {
        BOOST_TEST(slamService.GetNextDataFileOnline() ==
                   (slamService.path_to_data + "/" + data_files[i - 1]));
        io::RemoveFile(slamService.path_to_data + "/" + data_files[i]);
    }

    // Remove the temporary directory and its contents
    utils::removeTmpDirectory(tmp_dir);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace
}  // namespace viam
