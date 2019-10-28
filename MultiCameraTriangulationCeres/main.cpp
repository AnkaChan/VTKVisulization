#include "TriangulationCost.h"
#include "AC/Parser.h"
#include "AC/IO_AC.h"
#include <nlohmann/json.hpp>
#include <fstream>

#define CORNER_KEYS_NON_HDF5
#include "../../ToolBox/CornerPatternExtraction/UniqueCornerIdGenerator.h"
#include "../../ToolBox/Reconstruct/UniqueIdMatcher.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct TriangulationConfig
{
	double reprojThreshold = 0.5;
	int numCams = 8;
	double functionTolerance = 1e-3;
	bool minimizerOuput = false;
	bool fullOutput = false;
	bool undist = false;
	bool outputOff = false;
	bool k3 = false;
	std::string uIdSetFile = "C:\\Code\\MyRepo\\ChbCapture\\04_Pipeline\\GenerateModelSequenceMesh7\\CID_no_meshVID.txt";
};

void parseArgs(int argc, char** argv, TriangulationConfig& config) {
	std::string modeStr;
	cxxopts::Options options(argv[0], "App generate correspondences and triangles from label files.");

	options.add_options()
		("f,functionTolerance", "[Optional]Do a fitting info print every how many frames.", cxxopts::value<double>(config.functionTolerance))
		("r,reprojThreshold", "[Optional]Do a fitting info print every how many frames.", cxxopts::value<double>(config.reprojThreshold))
		("n,numCams", "[Optional]Do a fitting info print every how many frames.", cxxopts::value<int>(config.numCams))
		("m,minimizerOuput", "[Optional]Do a fitting info print every how many frames.", cxxopts::value<bool>(config.minimizerOuput))
		("o,fullOutput", "[Optional]Do a fitting info print every how many frames.", cxxopts::value<bool>(config.fullOutput))
		("u,undist", "...", cxxopts::value<bool>(config.undist))
		("outputOff", "...", cxxopts::value<bool>(config.outputOff))
		("k,k3", "...", cxxopts::value<bool>(config.k3))
		;

	try
	{
		auto result = options.parse(argc, argv);
	}
	catch (const cxxopts::OptionException& e)
	{
		std::cout << "error parsing options: " << e.what() << "\n";
		std::cout << options.help();
		exit(1);
	}
}

void getTriangles(std::vector<double*>& points, UniqueIdset& uIdSet, std::vector<Triangle>& tVec, bool realData = true) {

#ifdef OUTPUT_DEBUG_FILE
	std::ofstream of("debugFile.obj");
	for (int i = 0; i < pCMerged->points.size(); i++)
	{
		PType& pp = pCMerged->points[i];
		of << "v " << pp.x << " " << pp.y << " " << pp.z;
		of << " # " << UniqueCornerIdGenerator::keysToString(uIdSet.getUniqueCornersVec()[i]) << "\n";
	}
	//of << "# " << "White Patterns: \n";
#endif
		//Add the white pattern
	std::set<std::string> codeSet;
	for (int i = 0; i < uIdSet.getUniqueCornersVec().size(); i++)
	{
		CornerWithKey& c = uIdSet.getUniqueCornersVec()[i];
		if (c.keys.size() == 1)
		{
			codeSet.insert(c.keys.front().code);
		}
		else
		{
			codeSet.insert(c.keys[0].code);
			codeSet.insert(c.keys[1].code);
		}
	}

	for (auto code : codeSet)
	{
		CornerKey ck1, ck2, ck3;
		ck1.code = ck2.code = ck3.code = code;
		ck1.codeIndex = 0;
		ck2.codeIndex = 3;
		ck3.codeIndex = 2;

		Triangle t;
		t.id[0] = uIdSet.getKeyId(ck1);
		t.id[1] = uIdSet.getKeyId(ck2);
		t.id[2] = uIdSet.getKeyId(ck3);

		if (t.id[0] != -1 && t.id[1] != -1 && t.id[2] != -1)
		{
			if (points[t.id[0]][2] > 0
				&& points[t.id[1]][2] > 0
				&& points[t.id[2]][2] > 0)
			{
				tVec.push_back(t);
#ifdef OUTPUT_DEBUG_FILE
				of << "f " << t.id[0] + 1 << " " << t.id[1] + 1 << " " << t.id[2] + 1;
				of << " # White Pattern From Code: " << code << "\n";
#endif
			}
			else if (!realData)
			{
				tVec.push_back(t);
			}
		}

		ck1.codeIndex = 0;
		ck2.codeIndex = 2;
		ck3.codeIndex = 1;

		t.id[0] = uIdSet.getKeyId(ck1);
		t.id[1] = uIdSet.getKeyId(ck2);
		t.id[2] = uIdSet.getKeyId(ck3);


		if (t.id[0] != -1 && t.id[1] != -1 && t.id[2] != -1)
		{
			if (points[t.id[0]][2] > 0
				&& points[t.id[1]][2] > 0
				&& points[t.id[2]][2] > 0)
			{
				tVec.push_back(t);
#ifdef OUTPUT_DEBUG_FILE
				of << "f " << t.id[0] + 1 << " " << t.id[1] + 1 << " " << t.id[2] + 1;
				of << " # White Pattern From Code: " << code << "\n";
#endif
			}
			else if (!realData)
			{
				tVec.push_back(t);
			}
		}


	}

	std::vector<CornerWithKey> dualKeys;
	for (int i = 0; i < uIdSet.getUniqueCornersVec().size(); i++)
	{
		CornerWithKey& c = uIdSet.getUniqueCornersVec()[i];
		if (c.keys.size() == 2)
		{
			dualKeys.push_back(c);
		}
	}
	//Add the black pattern
	for (CornerWithKey code : dualKeys)
	{
		if ((code.keys[0].codeIndex != 0 && code.keys[1].codeIndex != 2)
			&& (code.keys[0].codeIndex != 2 && code.keys[1].codeIndex != 0))
		{
			continue;
		}
		//Switch to preserve the orientation
		if (code.keys[0].codeIndex == 2 && code.keys[1].codeIndex == 0) {
			CornerKey temp = code.keys[1];
			code.keys[1] = code.keys[0];
			code.keys[0] = temp;

		}
		CornerKey ck1, ck2, ck3;
		ck1.code = code.keys[0].code;
		ck2 = code.keys[1];
		ck3.code = code.keys[1].code;

		ck1.codeIndex = 3;
		ck3.codeIndex = 3;

		Triangle t;
		t.id[0] = uIdSet.getKeyId(ck1);
		t.id[1] = uIdSet.getKeyId(ck2);
		t.id[2] = uIdSet.getKeyId(ck3);

		if (t.id[0] != -1 && t.id[1] != -1 && t.id[2] != -1)
		{
			if (points[t.id[0]][2] > 0
				&& points[t.id[1]][2] > 0
				&& points[t.id[2]][2] > 0)
			{
				tVec.push_back(t);

#ifdef OUTPUT_DEBUG_FILE
				of << "f " << t.id[0] + 1 << " " << t.id[1] + 1 << " " << t.id[2] + 1;
				of << " # Black Pattern From Code: " << UniqueCornerIdGenerator::keysToString(code) << "\n";
#endif
			}
			else if (!realData)
			{
				tVec.push_back(t);
			}
		}

		ck1.code = code.keys[0].code;
		ck2.code = code.keys[1].code;
		ck3 = code.keys[1];

		ck1.codeIndex = 1;
		ck2.codeIndex = 1;

		t.id[0] = uIdSet.getKeyId(ck1);
		t.id[1] = uIdSet.getKeyId(ck2);
		t.id[2] = uIdSet.getKeyId(ck3);

		if (t.id[0] != -1 && t.id[1] != -1 && t.id[2] != -1)
		{
			if (points[t.id[0]][2] > 0
				&& points[t.id[1]][2] > 0
				&& points[t.id[2]][2] > 0)
			{
				tVec.push_back(t);

#ifdef OUTPUT_DEBUG_FILE
				of << "f " << t.id[0] + 1 << " " << t.id[1] + 1 << " " << t.id[2] + 1;
				of << " # Black Pattern From Code: " << UniqueCornerIdGenerator::keysToString(code) << "\n";
#endif
			}
			else if (!realData)
			{
				tVec.push_back(t);
			}
		}
	}

}

void loadCorrsFile(const std::string inCorrsFile, CorrPtsSet& corrs, Pts3dSet& initialTriangulation) {
	std::ifstream is(inCorrsFile);
	nlohmann::json j;
	is >> j;
	corrs = j["CorrPts"].get<CorrPtsSet>();
	initialTriangulation = j["initialTriangulation"].get<Pts3dSet>();
	
}

void writeCloudToObj(const std::string fileName, std::vector<double*>& points3D, const std::vector<Triangle>& tVec) {

	std::ofstream ofsObj(fileName);
	AC::IO::FileParts fp = AC::IO::fileparts(fileName);
	std::string mtlFileName = fp.name + ".mtl";

	cv::Mat errMat(1, 1, CV_8UC1);
	cv::Mat outColorMat(1, 1, CV_8UC3);
	for (int i = 0; i < points3D.size(); i++)
	{
		auto& p = points3D[i];

		ofsObj << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";


	}

	for (int i = 0; i < tVec.size(); i++)
	{
		ofsObj << "f " << tVec[i].id[0] + 1 << " " << tVec[i].id[1] + 1 << " " << tVec[i].id[2] + 1 << "\n";
	}

	ofsObj.close();
}

void writeCloudToPLY(const std::string& fileName, std::vector<double*>& points3D) {
	std::ofstream ofsPLY(fileName);

	ofsPLY
		<< "ply\nformat ascii 1.0\ncomment Mocap generated\n"
		<< "element vertex " << points3D.size() << "\n"
		<< "property float x\nproperty float y\nproperty float z\nend_header\n";


	for (int i = 0; i < points3D.size(); i++)
	{

		ofsPLY << points3D[i][0] << points3D[i][1] << points3D[i][2] << "\n";
	}

	ofsPLY.close();
}

int main(int argc, char** argv) {
	std::string inCorrsFile = argv[1];
	std::string inParamsFile = argv[2];
	std::string outFile = argv[3];

	TriangulationConfig cfg;
	parseArgs(argc, argv, cfg);

	CorrPtsSet corrs;
	Pts3dSet initialTriangulation;
	loadCorrsFile(inCorrsFile, corrs, initialTriangulation);

	std::vector<double*> world3dPts;

	for (size_t i = 0; i < initialTriangulation.size(); i++)
	{
		world3dPts.push_back(new double[3]);
		world3dPts[i][0] = initialTriangulation[i][0];
		world3dPts[i][1] = initialTriangulation[i][1];
		world3dPts[i][2] = initialTriangulation[i][2];
		//world3dPts[i][0] = 0;
		//world3dPts[i][1] = 0;
		//world3dPts[i][2] = 0;
	}

	CameraParameters** cp = new CameraParameters*[cfg.numCams];

	CameraParameters::loadCameraParameters(inParamsFile.c_str(), cp, cfg.numCams);

	Solver::Summary summary;
	Solver::Options options;
	options.minimizer_progress_to_stdout = cfg.minimizerOuput;

	options.function_tolerance = cfg.functionTolerance;
	//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	Problem problem;

	ceres::LossFunctionWrapper* loss_function = new ceres::LossFunctionWrapper(new ceres::CauchyLoss(cfg.reprojThreshold), ceres::TAKE_OWNERSHIP);

	std::vector<double> errsInitial(1487, 0.);
	std::vector<double> errsFinal(1487, 0.);
	std::vector<std::vector<ceres::CostFunction*>> costFuncs;

	for (size_t i = 0; i < initialTriangulation.size(); i++)
	{
		if (world3dPts[i][2] < 0)
		{
			continue;
		}
		//std::cout << "Id " << i << " | ";
		for (size_t iCam = 0; iCam < cfg.numCams; ++iCam) {
			//std::cout << corrs[iCam][i][0] << ", " << corrs[iCam][i][1] ;
			if (corrs[iCam][i][0] > 0 && corrs[iCam][i][1] > 0)
			{
				ceres::CostFunction* pCost;
				if (cfg.undist)
				{
					pCost = ReprojectionErrorUndist::Create(corrs[iCam][i][0], corrs[iCam][i][1], cp[iCam], !cfg.k3);
				}
				else
				{
					pCost = ReprojectionErrorNoUndist::Create(corrs[iCam][i][0], corrs[iCam][i][1], cp[iCam]);
				}
				double res[2];
				double** paramBlock = &world3dPts[i];
				pCost->Evaluate(paramBlock, res, NULL);
				//std::cout << "Residual: " << res[0] << ", " << res[1];
 				problem.AddResidualBlock(pCost, loss_function, world3dPts[i]);
			}
			//std::cout << " | ";
		}
		/*std::cout << "\n";
		if (i == 769  || i == 808 || i == 809 || i == 810 || i == 811 || i == 919 || i == 945 || i == 946)
		{
			std::cout << "Gocha\n";
		}*/
	}
	Solve(options, &problem, &summary);
	if (!cfg.outputOff)
	{
		if (cfg.fullOutput)
		{
			std::cout << summary.FullReport() << std::endl;
		}
		else
		{
			std::cout << summary.BriefReport() << std::endl;
		}
	}
	

	AC::IO::FileParts fp = AC::IO::fileparts(outFile);
	if (fp.ext == ".ply" || fp.ext == ".PLY")
	{
		writeCloudToPLY(outFile, world3dPts);
	}
	else if (fp.ext == ".obj" || fp.ext == ".Obj")
	{
		UniqueIdset uIdSet;
		uIdSet.loadUniqueIdSet(cfg.uIdSetFile);
		//getTriangles(points3D, matcher.uIdSet, tVec);
		std::vector<Triangle> tVec;
		getTriangles(world3dPts, uIdSet, tVec);
		writeCloudToObj(outFile, world3dPts, tVec);
	}
	else
	{
		std::cout << "Unsupported output format: " << fp.ext;
		return -1;
	}

}