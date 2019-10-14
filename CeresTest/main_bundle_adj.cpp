#if 0
#define  _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
using std::cout;
using std::endl;

#define LEN_IMAGE_NAME 6
#define NUM_CAMERA 8

// camera properties
static bool* g_chb_detected; // 8 x num_frames
class TriangulationProblem {
public:
	~TriangulationProblem() {
		delete[] g_chb_detected;
		delete[] image_point_measurements_;
		delete[] img_names_;
	}
	const double* image_point_measurements() const {
		return image_point_measurements_;
	}
	const bool* chb_detected() const {
		return g_chb_detected;
	}
	int num_frames() const {
		return num_frames_;
	}

	bool LoadDataset(const char* filename) {
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL) {
			return false;
		}

		FscanfOrDie(fptr, "%d", &num_frames_);

		g_chb_detected = new bool[NUM_CAMERA * NUM_POINTS * num_frames_];
		image_point_measurements_ = new double[NUM_CAMERA * NUM_POINTS * 2 * num_frames_];
		img_names_ = (char **)malloc((LEN_IMAGE_NAME + 1) * num_frames_ * sizeof(char*));

		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			// for each frame
			FscanfOrDie(fptr, "%s", &img_name_[0]);
			FscanfOrDie(fptr, "%d", &num_chb_cameras_);
			img_names_[frame_idx] = (char*)malloc((LEN_IMAGE_NAME + 1) * sizeof(char));
			strcpy(img_names_[frame_idx], img_name_);
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				FscanfOrDie(fptr, "%d", &curr_camera_idx_);
				FscanfOrDie(fptr, "%lf", &img_x1);
				FscanfOrDie(fptr, "%lf", &img_y1);
				FscanfOrDie(fptr, "%lf", &img_x2);
				FscanfOrDie(fptr, "%lf", &img_y2);
				FscanfOrDie(fptr, "%lf", &img_x3);
				FscanfOrDie(fptr, "%lf", &img_y3);
				g_chb_detected[(frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + 0] = !(img_x1 < 0.0);
				g_chb_detected[(frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + 1] = !(img_x2 < 0.0);
				g_chb_detected[(frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + 2] = !(img_x3 < 0.0);
				image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 0] = img_x1;
				image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 1] = img_y1;
				image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 2] = img_x2;
				image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 3] = img_y2;
				image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 4] = img_x3;
				image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 5] = img_y3;
			}
		}

		// check data input
		/*
		for (int frame_idx = 0; frame_idx < num_frames; frame_idx++) {
			printf("[%d] ===============================\n", frame_idx);
			for (int cam_idx = 0; cam_idx < kNumCameras; cam_idx++) {
				bool dtd = chb_detected[kNumCameras * frame_idx + cam_idx];
				if (dtd > 0) {
					double x = image_points[kNumCameras * 2 * frame_idx + 2 * cam_idx + 0];
					double y = image_points[kNumCameras * 2 * frame_idx + 2 * cam_idx + 1];
					printf("cam[%d]: (%lf, %lf)\n", cam_idx, x, y);
				}
				else {
					printf("cam[%d]:\n", cam_idx);
				}
			}
			printf("\n");
		}
		*/
		return true;
	}

private:
	template<typename T>
	void FscanfOrDie(FILE *fptr, const char* format, T *value) {
		int num_scanned = fscanf(fptr, format, value);
		if (num_scanned != 1) {
			LOG(FATAL) << "invalid UW data file.";
		}
	}
	char img_name_[LEN_IMAGE_NAME + 1];
	int num_chb_cameras_;
	int num_frames_;
	bool detected_;

	int curr_camera_idx_;
	double img_x1, img_y1, img_x2, img_y2, img_x3, img_y3;

	double* image_point_measurements_; // 2 x 8 x num_frames
	char** img_names_;
};
struct ReprojectionError_A {
	double observed_x, observed_y; // size 8
	double ang_axis_[3];
	double trans_[3];
	double focal_length_;
	double k_[2];
	double cx, cy;

	ReprojectionError_A(double observed_xi, double observed_yi) : observed_x(observed_xi), observed_y(observed_yi) {
		ang_axis_[0] = 1.4661068;
		ang_axis_[1] = 1.3909396;
		ang_axis_[2] = -1.0101221;
		trans_[0] = 120.72079937380548;
		trans_[1] = 878.5098164911466;
		trans_[2] = 2141.9479223664157;
		focal_length_ = 1892.5004985141147;
		k_[0] = -0.042601304678812096;
		k_[1] = 0.007006407740408107;
		cx = 1966.8652026818509;
		cy = 1083.233114758923;
	}
	template<typename T>
	bool operator()(const T* const world_point, T* residuals) const {
		const T ang_axis[3] = { T(ang_axis_[0]), T(ang_axis_[1]),T(ang_axis_[2]) };
		const T trans[3] = { T(trans_[0]),T(trans_[1]),T(trans_[2]) };
		const T focal_length = T(focal_length_);
		const T k[2] = { T(k_[0]), T(k_[1]) };

		T p[3];
		// camera_parameters[0, 1, 2] = angle-axis rotation
		ceres::AngleAxisRotatePoint(ang_axis, world_point, p);

		// camera_parameters[3, 4, 5]: translation
		p[0] += trans[0];
		p[1] += trans[1];
		p[2] += trans[2];

		// center of distortion
		T xp = p[0] / p[2];
		T yp = p[1] / p[2];

		T r2 = xp * xp + yp * yp;
		T distortion = 1.0 + r2 * (k[0] + k[1] * r2);

		// projected point position
		T predicted_x = focal_length * distortion * xp + cx;
		T predicted_y = focal_length * distortion * yp + cy;

		// error
		T dx = predicted_x - observed_x;
		T dy = predicted_y - observed_y;

		// output
		residuals[0] = dx;
		residuals[1] = dy;
		return true;
	}
	static ceres::CostFunction* Create(double observed_x, double observed_y) {
		// 2: residual dim
		// 3: image point dim (what we want to optimize)
		return (new ceres::AutoDiffCostFunction<ReprojectionError_A, 2, 3>(new ReprojectionError_A(observed_x, observed_y)));
	}
};
int main(int argc, char** argv) {
	google::InitGoogleLogging(argv[0]);

	const char* dataset_file_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib/_dataset_triangulation.txt";
	TriangulationProblem triangulation_problem;
	if (!triangulation_problem.LoadDataset(dataset_file_path)) {
		std::cerr << "ERROR: unable to open file " << dataset_file_path << "\n";
		std::getchar();
		return 1;
	}
	else {
		cout << "Done parsing: " << dataset_file_path << endl;
	}

	std::ofstream output_txt;
	output_txt.open("world_points_est.txt");

	const double* measurements = triangulation_problem.image_point_measurements();
	const int size_1d = NUM_CAMERA * NUM_POINTS*triangulation_problem.num_frames();
	double *measurements_x = new double[size_1d];
	double *measurements_y = new double[size_1d];
	int min_frame = 0;
	int max_frame = triangulation_problem.num_frames();
	//int max_frame = min_frame + 1;
	for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
		for (int ci = 0; ci < NUM_CAMERA; ci++) {
			for (int pi = 0; pi < NUM_POINTS; pi++) {
				int idx1 = (frame_idx * NUM_CAMERA + ci) * NUM_POINTS + pi;
				int idx2 = ((frame_idx * NUM_CAMERA + ci) * NUM_POINTS + pi) * 2;
				measurements_x[idx1] = measurements[idx2 + 0];
				measurements_y[idx1] = measurements[idx2 + 1];
			}
		}
	}

	//const bool* chb_detected = triangulation_problem.chb_detected();
	//for (int frame_idx = 06365; frame_idx < triangulation_problem.num_frames(); frame_idx++) {
	for (int frame_idx = min_frame; frame_idx < max_frame; frame_idx++) {
		printf("[%d]\n", frame_idx);

		int cam_used = 0;
		double world_point_output[NUM_POINTS][3] = { {0} };
		for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
			ceres::Problem problem;
			if (frame_idx > max_frame) {
				break;
			}
			double world_point[3] = { 0, 0, 0 };

			int measurement_idx = NUM_CAMERA * NUM_POINTS*frame_idx + world_point_idx;
			int chb_idx = frame_idx * NUM_CAMERA * NUM_POINTS + world_point_idx;
			if (g_chb_detected[chb_idx + NUM_POINTS * 0]) {
				ceres::CostFunction* cost_function = ReprojectionError_A::Create(measurements_x[measurement_idx + NUM_POINTS * 0], measurements_y[measurement_idx + NUM_POINTS * 0]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}
			if (g_chb_detected[chb_idx + NUM_POINTS * 1]) {
				ceres::CostFunction* cost_function = ReprojectionError_B::Create(measurements_x[measurement_idx + NUM_POINTS * 1], measurements_y[measurement_idx + NUM_POINTS * 1]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}
			if (g_chb_detected[chb_idx + NUM_POINTS * 2]) {
				ceres::CostFunction* cost_function = ReprojectionError_C::Create(measurements_x[measurement_idx + NUM_POINTS * 2], measurements_y[measurement_idx + NUM_POINTS * 2]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}
			if (g_chb_detected[chb_idx + NUM_POINTS * 3]) {
				ceres::CostFunction* cost_function = ReprojectionError_D::Create(measurements_x[measurement_idx + NUM_POINTS * 3], measurements_y[measurement_idx + NUM_POINTS * 3]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}
			if (g_chb_detected[chb_idx + NUM_POINTS * 4]) {
				ceres::CostFunction* cost_function = ReprojectionError_E::Create(measurements_x[measurement_idx + NUM_POINTS * 4], measurements_y[measurement_idx + NUM_POINTS * 4]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}
			if (g_chb_detected[chb_idx + NUM_POINTS * 5]) {
				ceres::CostFunction* cost_function = ReprojectionError_F::Create(measurements_x[measurement_idx + NUM_POINTS * 5], measurements_y[measurement_idx + NUM_POINTS * 5]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}
			if (g_chb_detected[chb_idx + NUM_POINTS * 6]) {
				ceres::CostFunction* cost_function = ReprojectionError_G::Create(measurements_x[measurement_idx + NUM_POINTS * 6], measurements_y[measurement_idx + NUM_POINTS * 6]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}
			if (g_chb_detected[chb_idx + NUM_POINTS * 7]) {
				ceres::CostFunction* cost_function = ReprojectionError_H::Create(measurements_x[measurement_idx + NUM_POINTS * 7], measurements_y[measurement_idx + NUM_POINTS * 7]);
				problem.AddResidualBlock(cost_function, NULL, world_point);
				cam_used++;
			}

			if (cam_used > 0) {
				ceres::Solver::Options options;
				options.linear_solver_type = ceres::DENSE_SCHUR;
				options.minimizer_progress_to_stdout = false;

				ceres::Solver::Summary summary;
				ceres::Solve(options, &problem, &summary);
				//cout << summary.FullReport() << endl;
				//cout << "Estimated world point = (" << world_point[0] << ", " << world_point[1] << ", " << world_point[2] << ")" << endl;
				//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				world_point_output[world_point_idx][0] = world_point[0];
				world_point_output[world_point_idx][1] = world_point[1];
				world_point_output[world_point_idx][2] = world_point[2];
			}
		}


		if (int(cam_used / 3) > 1) {
			output_txt << frame_idx << " ";
			for (int c = 0; c < NUM_CAMERA; c++) {
				output_txt << g_chb_detected[frame_idx * NUM_CAMERA * NUM_POINTS + c * NUM_POINTS] << " ";
			}

			for (int pi = 0; pi < NUM_POINTS; pi++) {
				output_txt << world_point_output[pi][0] << " " << world_point_output[pi][1] << " " << world_point_output[pi][2] << " ";
				//printf(" (%.2f, %.2f, %.2f) ", world_point_output[pi][0], world_point_output[pi][1], world_point_output[pi][2]);
			}
			output_txt << "\n";
			//printf("\n");
		}
	}

	output_txt.close();
	printf("Done");
	std::getchar();

	delete[] measurements_x;
	delete[] measurements_y;
	return 0;
}

#endif
