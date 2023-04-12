#include <Eigen/Core>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <cassert>
#include <limits>
#include <cmath>
#include <algorithm>
#include <CLI/CLI.hpp>
#include <filesystem>

#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/view.h"

#include <igl/file_dialog_open.h>
#include <tbb/tbb.h>

bool isFirstRun = true;
std::vector<Eigen::MatrixXd> posList;
std::vector<Eigen::MatrixXi> faceList;

int curFrame = 0;
int numFrames = 1;
int startId = 0;
bool isSameMesh = true;

std::string inputPath;

static void mkdir(const std::string& foldername)
{
    if (!std::filesystem::exists(foldername))
    {
        std::cout << "create directory: " << foldername << std::endl;
        if (!std::filesystem::create_directory(foldername))
        {
            std::cerr << "create folder failed." << foldername << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}

bool loadProblem(std::string loadingPath = "")
{
    std::cout << "load path: " << loadingPath << std::endl;
    if(loadingPath == "")
        loadingPath = igl::file_dialog_open();
    std::replace(loadingPath.begin(), loadingPath.end(), '\\', '/'); // handle the backslash issue for windows
    int id = loadingPath.rfind("/");
    std::string folder = loadingPath.substr(0, id + 1); // include "/"
    std::string meshName = loadingPath.substr(id + 1, loadingPath.size() - id);
    std::cout << meshName << std::endl;

    std::string meshNamePrefix = "";
    for(auto& ch : meshName)
    {
        if(!std::isdigit(ch))
        {
            meshNamePrefix.push_back(ch);
        }
        else
        {
            std::cout << "prefix: " << meshNamePrefix << std::endl;
            break;
        }
    }

    for(numFrames = 0;;)
    {
        if(std::filesystem::exists(folder + meshNamePrefix + std::to_string(numFrames + startId) + ".obj"))
        {

            numFrames++;
        }
        else
        {
            std::cout << "total frames: " << numFrames << std::endl;
            break;
        }
    }

    if(numFrames == 0)
    {
        std::cout << "no frames are found" << std::endl;
        exit(EXIT_FAILURE);
    }

    posList.resize(numFrames);
    faceList.resize(numFrames);

    for (uint32_t i = 0; i < numFrames; ++i)
    {
        std::cout << "load frame: " << i << std::endl;
        igl::readOBJ(folder + meshNamePrefix + std::to_string(i + startId) + ".obj", posList[i], faceList[i]);
    }

   /* auto loadPerFrame = [&](const tbb::blocked_range<uint32_t>& range)
    {
        for (uint32_t i = range.begin(); i < range.end(); ++i)
        {
            std::cout << "load frame: " << i << std::endl;
            igl::readOBJ(folder + meshNamePrefix + std::to_string(i + startId) + ".obj", posList[i], faceList[i]);
        }
    };
    tbb::blocked_range<uint32_t> rangex(0u, (uint32_t)numFrames, 1);
    tbb::parallel_for(rangex, loadPerFrame);*/
    std::cout << "load mesh finished!" << std::endl;
    inputPath = loadingPath;

    if(numFrames > 1)
    {
        isSameMesh = (faceList[0] - faceList[1]).norm() == 0;
    }
    isFirstRun = true;
    curFrame = 0;

    return true;
}

void updateView(int frameId)
{
    //std::cout << "update viewer. " << std::endl;
    if (isFirstRun || !isSameMesh)
    {
        polyscope::registerSurfaceMesh("mesh", posList[frameId], faceList[frameId]);
    }
    else
    {
        polyscope::getSurfaceMesh("mesh")->updateVertexPositions(posList[frameId]);
    }

    isFirstRun = false;
}


void callback() {
    ImGui::PushItemWidth(100);
    if (ImGui::Button("load objs", ImVec2(-1, 0)))
    {
        if(loadProblem())
            updateView(curFrame);
    }
    if (ImGui::InputInt("start frame", &startId))
    {

    }
    if (ImGui::CollapsingHeader("Frame Visualization Options", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::SliderInt("current frame", &curFrame, 0, numFrames - 1))
        {
            curFrame = curFrame % numFrames;
            updateView(curFrame);
        }
    }

    if (ImGui::Button("output images", ImVec2(-1, 0)))
    {
        int id = inputPath.rfind("/");
        std::string preFolder = inputPath.substr(0, id);
        std::cout << "save folder: " << preFolder << std::endl;
        std::string subFolder = "/polyimags/";
        mkdir(preFolder + subFolder);

        for (int i = 0; i < numFrames; i++)
        {
            updateView(i);
            //polyscope::options::screenshotExtension = ".jpg";
            std::string name = preFolder + subFolder + "/output_" + std::to_string(i) + ".png";
            polyscope::screenshot(name);
        }
    }

    if (ImGui::Button("output meshes", ImVec2(-1, 0)))
    {
        int id = inputPath.rfind("/");
        std::string preFolder = inputPath.substr(0, id);
        std::cout << "save folder: " << preFolder << std::endl;
        std::string subFolder = "/meshSequence/";
        mkdir(preFolder + subFolder);

        for (int i = 0; i < numFrames; i++)
        {
            std::string name = preFolder + subFolder + "/mesh_" + std::to_string(i) + ".obj";
            igl::writeOBJ(name, posList[i], faceList[i]);
        }
    }


    ImGui::PopItemWidth();
}

int main(int argc, char *argv[])
{
    std::string defaultPath;
    CLI::App app("cylinder analysis");
    app.add_option("input,-i,--input", defaultPath, "Input mesh (obj file)")->check(CLI::ExistingFile);
    app.add_option("-s,--start", startId, "Start id");

    try
    {
        app.parse(argc, argv);
    }
    catch (const CLI::ParseError& e)
    {
        return app.exit(e);
    }

    // Options
//	polyscope::options::autocenterStructures = true;
    polyscope::view::windowWidth = 1024;
    polyscope::view::windowHeight = 1024;

    // Initialize polyscope
    polyscope::init();

    polyscope::view::upDir = polyscope::view::UpDir::ZUp;

    // Add the callback
    polyscope::state::userCallback = callback;

    polyscope::options::groundPlaneHeightFactor = 0.25; // adjust the plane height

    if(loadProblem(defaultPath))
        updateView(curFrame);

    // Show the gui
    polyscope::show();

    return 0;
}