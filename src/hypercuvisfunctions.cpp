#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include "hyperfunctions.cpp"
#include "hyperfunctions.h"
#include "hypercuvisfunctions.h"
#include "ctpl.h"
#include "cuvis.hpp"
#include <cassert>
#include <iostream>

using namespace cv;
using namespace std;
using namespace std::chrono;

// Loads first hyperspectral image for analysis
void HyperFunctionsCuvis::LoadImageHyper(string file_name, bool isImage1 = true)
{
    if (isImage1) {
        mlt1.clear();
    }
    else {
        mlt2.clear();
    }
	string file_ext;

    size_t dotPos = file_name.find_last_of('.');
    if (dotPos != std::string::npos) {
        file_ext = file_name.substr(dotPos + 1);
    }
   
    // cout<<"extension is: "<<file_ext<<endl; 

    if (file_ext=="cu3")
    {
        char* const userSettingsDir =  const_cast<char*>( cubert_settings.c_str());
        char* const measurementLoc =  const_cast<char*>( file_name.c_str());

        cuvis::General::init(userSettingsDir);
        // cuvis::General::set_log_level(loglevel_info);

        std::cout << "loading measurement file..." << std::endl;
        cuvis::Measurement mesu(measurementLoc);

         if (mesu.get_meta()->measurement_flags.size() > 0)
        {
            std::cout << "  Flags" << std::endl;
            for (auto const& flags : mesu.get_meta()->measurement_flags)
            {
                std::cout << "  - " << flags.first << " (" << flags.second << ")" << std::endl;
            }
        }


        // assert(
        //     mesu.get_meta()->processing_mode == Cube_Raw &&
        //     "This example requires raw mode");
        if (!(mesu.get_meta()->processing_mode == Cube_Raw)) {
            throw std::runtime_error("This example requires raw mode");
        }

        auto const& cube_it = mesu.get_imdata()->find(CUVIS_MESU_CUBE_KEY);
        assert(
            cube_it != mesu.get_imdata()->end() &&
            "Cube not found");

        auto cube = std::get<cuvis::image_t<std::uint16_t>>(cube_it->second);

        cv::Mat img(
        cv::Size(cube._width, cube._height),
        CV_16UC(cube._channels),
        const_cast<void*>(reinterpret_cast<const void*>(cube._data)),
        cv::Mat::AUTO_STEP);

        for (int i=0;i<img.channels();i++)
        {
            cv::Mat singleChannel;
            cv::extractChannel(
            img, singleChannel, i); 
            singleChannel.convertTo(singleChannel, CV_8U, 1 / 16.0);
            if (isImage1) {
                mlt1.push_back(singleChannel);
            }
            else {
                mlt2.push_back(singleChannel);
            }
            // cv::imshow(" Individual channel ", singleChannel);
            // cv::waitKey(50);
        }         



    }
    else if (file_ext=="cu3s")
    {
        char* const userSettingsDir =  const_cast<char*>( cubert_settings.c_str());
        char* const measurementLoc =  const_cast<char*>( file_name.c_str());

        // std::cout << "loading user settings..." << std::endl;
        cuvis::General::init(userSettingsDir);
        // cuvis::General::set_log_level(loglevel_info);

        // std::cout << "loading session... " << std::endl;
        cuvis::SessionFile sess(measurementLoc);

        // std::cout << "loading measurement... " << std::endl;
        auto optmesu = sess.get_mesu(0);
        assert(optmesu.has_value());
        cuvis::Measurement mesu = optmesu.value();

        // std::cout << "Data 1" << mesu.get_meta()->name << " "
        //         << "t=" << mesu.get_meta()->integration_time << " ms "
        //         << "mode=" << mesu.get_meta()->processing_mode << " " << std::endl;
        if (mesu.get_meta()->measurement_flags.size() > 0)
        {
        std::cout << "  Flags" << std::endl;
        for (auto const& flags : mesu.get_meta()->measurement_flags)
        {
            std::cout << "  - " << flags.first << " (" << flags.second << ")"
                    << std::endl;
        }
        }

        // assert(
        //     mesu.get_meta()->processing_mode == Cube_Raw &&
        //     "This example requires raw mode");

        if (!(mesu.get_meta()->processing_mode == Cube_Raw)) {
            throw std::runtime_error("This example requires raw mode");
        }

        auto const& cube_it = mesu.get_imdata()->find(CUVIS_MESU_CUBE_KEY);
        assert(cube_it != mesu.get_imdata()->end() && "Cube not found");

        auto cube = std::get<cuvis::image_t<std::uint16_t>>(cube_it->second);

        cv::Mat img(
        cv::Size(cube._width, cube._height),
        CV_16UC(cube._channels),
        const_cast<void*>(reinterpret_cast<const void*>(cube._data)),
        cv::Mat::AUTO_STEP);

        for (int i=0;i<img.channels();i++)
        {
            cv::Mat singleChannel;
            cv::extractChannel(
            img, singleChannel, i); 
            singleChannel.convertTo(singleChannel, CV_8U, 1 / 16.0);
            if (isImage1) {
                mlt1.push_back(singleChannel);
            }
            else {
                mlt2.push_back(singleChannel);
            }
            // cv::imshow(" Individual channel ", singleChannel);
            // cv::waitKey(50);
        }         

    }
    else if (file_ext=="tiff")
    {
        if (isImage1) {
            imreadmulti(file_name, mlt1);
        }
        else {
            imreadmulti(file_name, mlt2);
        }
    }
    
    
    
}

void HyperFunctionsCuvis::TakeImageHyper1(string file_name, const int exposure_ms, const int num_images) {

    // Take image using cuvis and put it in the export directory

    char* const userSettingsDir = const_cast<char*>(cubert_settings.c_str());
    char* const factoryDir = const_cast<char*>(factory_dir.c_str());
    char* const recDir = const_cast<char*>(output_dir.c_str());

    // Loading user settings
    cuvis::General::init(userSettingsDir);
    // cuvis::General::set_log_level(loglevel_info);

    // std::cout << "Loading Calibration and processing context..." << std::endl;
    // Loading calibration and processing context
    cuvis::Calibration calib(factoryDir);
    cuvis::ProcessingContext proc(calib);
    cuvis::AcquisitionContext acq(calib);

    cuvis::SaveArgs saveArgs;
    saveArgs.allow_overwrite = true;
    saveArgs.export_dir = recDir;
    saveArgs.allow_session_file = true;

    cuvis::CubeExporter exporter(saveArgs);

    while (cuvis::hardware_state_t::hardware_state_offline == acq.get_state())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Camera is online" << std::endl;
    acq.set_operation_mode(cuvis::operation_mode_t::OperationMode_Software).get();
    acq.set_integration_time(exposure_ms).get();
    
    auto session = cuvis::SessionInfo();
    session.name = file_name; // this is the name of the session and the base, the images will be named as base_001.cu3s
    acq.set_session_info(session);
  
    std::cout << "Start recording now" << std::endl;
    for (int k = 0; k < num_images; k++)
    {
        std::cout << "Record image #" << k << "... (async) ";
        auto async_mesu = acq.capture();
        auto mesu_res = async_mesu.get(std::chrono::milliseconds(500));
        if (mesu_res.first == cuvis::async_result_t::done &&
            mesu_res.second.has_value())
        {
        auto& mesu = mesu_res.second.value();

        // sets single image name and saves as a .cu3
        // mesu.set_name("Test");
        // mesu.save(saveArgs);

        proc.apply(mesu);
        exporter.apply(mesu);

        std::cout << "done" << std::endl;
        }
        else
        {
        std::cout << "failed" << std::endl;
        }
    }

    //uncomment for recording in queue mode
    /* 
    for (int k = 0; k < num_images; k++)
    {
        std::cout << "Record image #" << k << "... (queue)";
        acq.capture_queue();
        auto mesu = acq.get_next_measurement(std::chrono::milliseconds(500));
        if (mesu)
        {
            proc.apply(mesu.value());
            exporter.apply(mesu.value());
            std::cout << "done" << std::endl;
        }
        else
        {
            std::cout << "failed" << std::endl;
        }
        std::cout << "done" << std::endl;
    }
    */
    std::cout << "finished." << std::endl;
    
}

//export cubert image to multipage tiff image
//assumes cu3s as input
void HyperFunctionsCuvis::ExportTiff()
{

 
    char* const sessionLoc  =  const_cast<char*>(cubert_img.c_str());
    char* const userSettingsDir =  const_cast<char*>(cubert_settings.c_str());
    char* const exportDir =  const_cast<char*>(output_dir.c_str());

    cuvis::General::init(userSettingsDir);
    // cuvis::General::set_log_level(loglevel_info);
    cuvis::SessionFile sess(sessionLoc);

    std::cout << "loading measurement... " << std::endl;
    auto optmesu = sess.get_mesu(0);
    assert(optmesu.has_value());
    cuvis::Measurement mesu = optmesu.value();

    cuvis::ProcessingArgs procArgs;   
    procArgs.processing_mode = cuvis::processing_mode_t::Cube_Reflectance;

    char* const factoryDir =  const_cast<char*>(factory_dir.c_str());
    cuvis::Calibration calib(factoryDir);
    cuvis::ProcessingContext proc(calib);
    proc.set_processingArgs(procArgs);
    
    // not sure of how to change the processing mode
    // right now it is just set to raw
    // maybe it is because calibration files are not in cu3s file

    // proc.apply(mesu);
    
    cout<<mesu.get_meta()->processing_mode<<" process mode "<<endl;
    
    
    assert(mesu.get_meta()->processing_mode != cuvis::processing_mode_t::Preview);
    {
        std::cout << "Export to Multi-Page Tiff" << std::endl;
        cuvis::TiffArgs args;
        char exportDirMulti[CUVIS_MAXBUF];
        strcpy(exportDirMulti, exportDir);
        strcat(exportDirMulti, "/multi");
        args.export_dir = exportDirMulti;
        args.format = cuvis::tiff_format_t::tiff_format_MultiPage;
        cuvis::TiffExporter exporter(args);
        exporter.apply(mesu);
     }

}


// this is required when raw processing is required
// requires calibration images to calculate reflectance
void HyperFunctionsCuvis::ReprocessImage(string file_name, bool isImage1 = true ){

    
     if (isImage1) {
        mlt1.clear();
    }
    else {
        mlt2.clear();
    }
	string file_ext;
    
    
    size_t dotPos = file_name.find_last_of('.');
    if (dotPos != std::string::npos) {
        file_ext = file_name.substr(dotPos + 1);
    }



    if (file_ext=="cu3")
    {

        char* const userSettingsDir =  const_cast<char*>(cubert_settings.c_str());
        char* const measurementLoc =  const_cast<char*>(file_name.c_str());
        char* const darkLoc =  const_cast<char*>(dark_img.c_str());
        char* const whiteLoc =  const_cast<char*>(white_img.c_str());
        char* const distanceLoc =  const_cast<char*>(dist_img.c_str());
        char* const factoryDir =  const_cast<char*>(factory_dir.c_str());
        char* const outDir =  const_cast<char*>(output_dir.c_str());

        cuvis::General::init(userSettingsDir);
        // uncomment below for verbose output from cuvis processing pipeline
        // cuvis::General::set_log_level(loglevel_info);

        cuvis::Measurement mesu(measurementLoc);
        cuvis::Measurement dark(darkLoc);
        cuvis::Measurement white(whiteLoc);
        cuvis::Measurement distance(distanceLoc);
        cuvis::Calibration calib(factoryDir);
        cuvis::ProcessingContext proc(calib);
        proc.set_reference(dark, cuvis::reference_type_t::Reference_Dark);
        proc.set_reference(white, cuvis::reference_type_t::Reference_White);
        proc.set_reference(distance, cuvis::reference_type_t::Reference_Distance);
        

        procArgs.processing_mode = cuvis::processing_mode_t::Cube_Reflectance;
        proc.set_processingArgs(procArgs);
                
        if (proc.is_capable(mesu, procArgs))
        {
            proc.apply(mesu);
        }
        

        auto const& cube_it = mesu.get_imdata()->find(CUVIS_MESU_CUBE_KEY);
        assert(
            cube_it != mesu.get_imdata()->end() &&
            "Cube not found");
        auto cube = std::get<cuvis::image_t<std::uint16_t>>(cube_it->second);
        cv::Mat img(
        cv::Size(cube._width, cube._height),
        CV_16UC(cube._channels),
        const_cast<void*>(reinterpret_cast<const void*>(cube._data)),
        cv::Mat::AUTO_STEP);

        for (int i=0;i<img.channels();i++)
        {
            cv::Mat singleChannel;
            cv::extractChannel(
            img, singleChannel, i); 
            singleChannel.convertTo(singleChannel, CV_8U, 1 / 16.0);
            if (isImage1) {
                mlt1.push_back(singleChannel);
            }
            else {
                mlt2.push_back(singleChannel);
            }
            // cv::imshow(" Individual channel ", singleChannel);
            // cv::waitKey(50);

        } 
    }
    else if (file_ext=="cu3s")
    {

        char* const userSettingsDir = const_cast<char*>(cubert_settings.c_str());
        char* const measurementLoc = const_cast<char*>(file_name.c_str());
        char* const darkLoc = const_cast<char*>(dark_img.c_str());
        char* const whiteLoc = const_cast<char*>(white_img.c_str());
        char* const distanceLoc = const_cast<char*>(dist_img.c_str());
        char* const outDir = const_cast<char*>(output_dir.c_str());


        std::cout << "Example 02 reprocess measurement cpp " << std::endl;
        std::cout << "User Settings Dir: " << userSettingsDir << std::endl;
        std::cout << "measurement file (.cu3s): " << measurementLoc << std::endl;
        std::cout << "dark file (.cu3s): " << darkLoc << std::endl;
        std::cout << "white file (.cu3s): " << whiteLoc << std::endl;
        std::cout << "distance file (.cu3s): " << distanceLoc << std::endl;
        std::cout << "output Dir: " << outDir << std::endl;


        std::cout << "loading settings... " << std::endl;
        cuvis::General::init(userSettingsDir);
        // cuvis::General::set_log_level(loglevel_info);


        std::cout << "loading measurement... " << std::endl;
        cuvis::SessionFile sessMesu(measurementLoc);


        auto optMesu = sessMesu.get_mesu(0);
        assert(optMesu.has_value());
        cuvis::Measurement mesu = optMesu.value();




        std::cout << "loading dark... " << std::endl;
        cuvis::SessionFile sessDark(darkLoc);
        auto optDark = sessDark.get_mesu(0);
        assert(optDark.has_value());
        cuvis::Measurement dark = optDark.value();




        std::cout << "loading white... " << std::endl;
        cuvis::SessionFile sessWhite(whiteLoc);
        auto optWhite = sessWhite.get_mesu(0);
        assert(optWhite.has_value());
        cuvis::Measurement white = optWhite.value();
        
        
        // used when measurement image instead of value 
        // std::cout << "loading distance... " << std::endl;
        // cuvis::SessionFile sessDistance(distanceLoc);
        // auto optDistance = sessDistance.get_mesu(0);
        // assert(optDistance.has_value());
        // cuvis::Measurement distance = optDistance.value();
        
        
        std::cout << "Data 1" << mesu.get_meta()->name << " "
                    << "t=" << mesu.get_meta()->integration_time << " ms "
                    << "mode=" << mesu.get_meta()->processing_mode << " " << std::endl;


        std::cout << "Loading processing context" << std::endl;
        cuvis::ProcessingContext proc(sessMesu);


        std::cout << "Set references" << std::endl;


        proc.set_reference(dark, cuvis::reference_type_t::Reference_Dark);
        proc.set_reference(white, cuvis::reference_type_t::Reference_White);
        // used when measurement image instead of value 
        // proc.set_reference(distance, cuvis::reference_type_t::Reference_Distance);
        
        // use below if you know distance (doesn't matter if larger than 6 meters) distance is in mm
        proc.calc_distance(dist_val_mm);


        cuvis::ProcessingArgs procArgs;
        cuvis::SaveArgs saveArgs;
        saveArgs.allow_overwrite = true;
        saveArgs.allow_session_file = true;
        saveArgs.allow_info_file = false;


        std::map<std::string, cuvis::processing_mode_t> target_modes = {
            //   {"Raw", cuvis::processing_mode_t::Cube_Raw},
            //   {"DS", cuvis::processing_mode_t::Cube_DarkSubtract},
            {"Ref", cuvis::processing_mode_t::Cube_Reflectance}
            //   {"RAD", cuvis::processing_mode_t::Cube_SpectralRadiance}
            };


        for (auto const& mode : target_modes)
        {
            procArgs.processing_mode = mode.second;
            if (proc.is_capable(mesu, procArgs))
            {
            std::cout << "processing to mode " << mode.first << std::endl;
            proc.set_processingArgs(procArgs);
            proc.apply(mesu);
            saveArgs.export_dir = std::filesystem::path(outDir) / mode.first;


            //   cuvis::CubeExporter exporter(saveArgs);
            //   exporter.apply(mesu);
            }
            else
            {
            std::cout << "cannot process to mode " << mode.first << std::endl;
            }
        }




            auto const& cube_it = mesu.get_imdata()->find(CUVIS_MESU_CUBE_KEY);
            assert(
                cube_it != mesu.get_imdata()->end() &&
                "Cube not found");
            auto cube = std::get<cuvis::image_t<std::uint16_t>>(cube_it->second);
        cv::Mat img(
            cv::Size(cube._width, cube._height),
            CV_16UC(cube._channels),
            const_cast<void*>(reinterpret_cast<const void*>(cube._data)),
            cv::Mat::AUTO_STEP);


            for (int i=0;i<img.channels();i++)
            {
                cv::Mat singleChannel;
                cv::extractChannel(
                img, singleChannel, i); 
                singleChannel.convertTo(singleChannel, CV_8U, 1 / 16.0);
                mlt1.push_back(singleChannel);
                // cv::imshow(" Individual channel ", singleChannel);
                // cv::waitKey(50);
            } 
    }

  std::cout << "finished." << std::endl;
}
