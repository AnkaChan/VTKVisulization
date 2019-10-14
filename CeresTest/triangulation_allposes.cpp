#include "triangulation_allposes.h"
#define NUM_POINTS 88
#define K3OFF
#define LOSS_FUNC
static bool* g_chb_detected; // 8 x num_frames
class TriangulationAllPosesProblem {
public:
	~TriangulationAllPosesProblem() {
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
		srand(time(NULL));
		FILE* fptr = fopen(filename, "r");
		if (fptr == NULL) {
			return false;
		}

		FscanfOrDie(fptr, "%d", &num_frames_);

		g_chb_detected = new bool[NUM_CAMERA * num_frames_];
		image_point_measurements_ = new double[NUM_CAMERA * NUM_POINTS * 2 * num_frames_](); // initialize to 0s
		img_names_ = (char **)malloc((LEN_IMAGE_NAME + 1) * num_frames_ * sizeof(char*));

		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			// for each frame
			FscanfOrDie(fptr, "%s", &img_name_[0]);
			img_names_[frame_idx] = (char*)malloc((LEN_IMAGE_NAME + 1) * sizeof(char));
			strcpy(img_names_[frame_idx], img_name_);

			FscanfOrDie(fptr, "%d", &num_chb_cameras_);

			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				FscanfOrDie(fptr, "%d", &curr_camera_idx_);

				int chb_detected;
				FscanfOrDie(fptr, "%d", &chb_detected);
				g_chb_detected[frame_idx * NUM_CAMERA + cam_idx] = (chb_detected == 1);

				if (chb_detected == 1) {
					for (int imgpoint_idx = 0; imgpoint_idx < NUM_POINTS; imgpoint_idx++) {
						double u, v;
						FscanfOrDie(fptr, "%lf", &u);
						FscanfOrDie(fptr, "%lf", &v);
						image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + imgpoint_idx) * 2 + 0] = u;
						image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + imgpoint_idx) * 2 + 1] = v;
					}
				}
				else {
				}
			}
		}

		/*
		// check data input
		for (int frame_idx = 0; frame_idx < num_frames_; frame_idx++) {
			if (frame_idx < 160 || frame_idx > 170) continue;
			printf("frame[%d] image_name: %s ===============================\n", frame_idx, img_names_[frame_idx]);
			for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				bool dtd = g_chb_detected[frame_idx * NUM_CAMERA + cam_idx];
				if (dtd > 0) {
					double u1 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 0];
					double v1 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS) * 2 + 1];

					double u2 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + 87) * 2 + 0];
					double v2 = image_point_measurements_[((frame_idx * NUM_CAMERA + cam_idx)*NUM_POINTS + 87) * 2 + 1];
					printf("cam[%d]: (%.4lf, %.4lf), (%.4lf, %.4lf)\n", cam_idx, u1, v1, u2, v2);
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
	char** img_names_;

private:
	template<typename T>
	void FscanfOrDie(FILE *fptr, const char* format, T *value) {
		int num_scanned = fscanf(fptr, format, value);
		if (num_scanned != 1) {
			LOG(FATAL) << "invalid UW data file.";
		}
	}
	char img_name_[LEN_IMAGE_NAME + 1];
	int num_frames_;
	bool detected_;

	int curr_camera_idx_;
	double img_x1, img_y1, img_x2, img_y2, img_x3, img_y3;

	double* image_point_measurements_; // 2 x 8 x num_frames
	int num_chb_cameras_; // how many cameras saw checkboard
};
struct ReprojectionError {
	double observed_x, observed_y; // size 8
	CameraParameters cp;
	ReprojectionError(double observed_xi, double observed_yi) : observed_x(observed_xi), observed_y(observed_yi), cp(CameraParameters()) {
	}

	template<typename T>
	bool operator()(const T* const world_point, T* residuals) const {
		const T ang_axis[3] = { T(cp.ax[0]), T(cp.ax[1]),T(cp.ax[2]) };
		const T trans[3] = { T(cp.trans[0]),T(cp.trans[1]),T(cp.trans[2]) };
		const T f[2] = { T(cp.fx), T(cp.fy) };
		const T k[3] = { T(cp.k1), T(cp.k2) , T(cp.k3) };
		const T p[2] = { T(cp.p1), T(cp.p2) };
		const T c[2] = { T(cp.cx), T(cp.cy) };

		T cam_point[3];
		// angle-axis rotation
		ceres::AngleAxisRotatePoint(ang_axis, world_point, cam_point);

		// translation
		cam_point[0] += trans[0];
		cam_point[1] += trans[1];
		cam_point[2] += trans[2];

		// center of distortion
		T xp = cam_point[0] / cam_point[2];
		T yp = cam_point[1] / cam_point[2];

		// distortions
		T r2 = xp * xp + yp * yp;
#ifdef K3OFF
		T radial_dist = 1.0 + r2 * (k[0] + k[1] * r2);
#else
		T radial_dist = 1.0 + r2 * (k[0] + k[1] * r2 + k[2] * r2*r2);
#endif
		T tangential_dist_x = T(2.0) * p[0] * xp*yp + p[1] * (r2 + T(2.0) * xp*xp);
		T tangential_dist_y = p[0] * (r2 + T(2.0) * yp*yp) + T(2.0) * p[1] * xp*yp;

		xp = xp * radial_dist + tangential_dist_x;
		yp = yp * radial_dist + tangential_dist_y;

		// projected point position
		T predicted_x = f[0] * xp + c[0];
		T predicted_y = f[1] * yp + c[1];

		// error
		T dx = predicted_x - observed_x;
		T dy = predicted_y - observed_y;
		// output
		residuals[0] = dx;
		residuals[1] = dy;
		return true;
	}
	static ceres::CostFunction* Create(double observed_x, double observed_y, CameraParameters &cp_in) {
		ReprojectionError *re = new ReprojectionError(observed_x, observed_y);
		re->cp = cp_in;
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(re));
	}
};
TriangulationAllPoses::TriangulationAllPoses() {
}
int TriangulationAllPoses::DoWork() {
	google::InitGoogleLogging("HJP");

	const char* dataset_file_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/OutputForCeres/Triangulation/dataset_triangulation_allposes.txt";
	TriangulationAllPosesProblem triangulation_problem;
	if (!triangulation_problem.LoadDataset(dataset_file_path)) {
		std::cerr << "ERROR: unable to open file " << dataset_file_path << "\n";
		std::getchar();
		return 1;
	}
	else {
		cout << "Done parsing: " << dataset_file_path << endl;
	}

	// load camera parameters from xml
	CameraParameters *cam_params[NUM_CAMERA];
	// before bundle adjustment
	//const char* camparam_txt_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/OutputForCeres/CalibGlobalWorldCentered/camera_parameters.txt";

	// after bundle adjustment
	const char* camparam_txt_path = "C:/Users/joont/OneDrive/HJ/PhD/PycharmProjects/MultiViewCalib_v2Reorganized/AfterBundleAdjustment/CalibGlobWorldCentered/camera_parameters7.txt";
	int res = LoadCameraParameters(camparam_txt_path, cam_params);

	std::ofstream output_txt;
	output_txt.open("output/triangulation7_output.txt");
	//output_txt.open("output/triangulation3_output.txt");

	// Check measurement exclusion
	const char* exc_txt_path = "C:/Users/joont/Documents/Visual Studio 2017/Projects/CeresTest_v2Reorganized/CeresTest/output/output_excluded_measurements7.txt";

	bool *measurement_excluded = NULL;
	int exclusion_exists = LoadMeasurementExclusion(exc_txt_path, measurement_excluded);


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

	//for (int frame_idx = 06365; frame_idx < triangulation_problem.num_frames(); frame_idx++) {
	double cost_sum_final = 0;
	double cost_sum_initial = 0;
	const int total_num_3d_points = max_frame * NUM_POINTS;
	int actual_num_3d_points = 0;
	int excluded_count = 0;
	for (int frame_idx = 0; frame_idx < max_frame; frame_idx++) {
		int remainder = frame_idx % 100;
		if (remainder == 0) {
			printf("[%d]\n", frame_idx);
		}

		int cam_used = 0;
		double world_point_output[NUM_POINTS][3] = { {0} };
		double final_costs[NUM_POINTS] = {};

		for (int world_point_idx = 0; world_point_idx < NUM_POINTS; world_point_idx++) {
			ceres::Problem problem;
			if (frame_idx > max_frame) {
				break;
			}
			// setting initial world point to the origin should work, because the world origin is always in the field of view of each camera.

			int rand_int = rand() % 100 + 1;
			double rand_x = double(rand_int);
			double rand_y = double(rand_int);
			double rand_z = double(rand_int);
			double world_point[3] = { rand_x, rand_y, rand_z };

			// Configure the loss function.
#ifdef LOSS_FUNC
	// delta: https://en.wikipedia.org/wiki/Huber_loss
			const double delta = 1;
			ceres::LossFunction* loss = new ceres::HuberLoss(delta);
#else
			ceres::LossFunction* loss = NULL;
#endif

			int cam_used_per_point = 0;
			int measurement_idx = NUM_CAMERA * NUM_POINTS*frame_idx + world_point_idx;
			int chb_idx = frame_idx * NUM_CAMERA * NUM_POINTS + world_point_idx;

			for (size_t cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
				if (g_chb_detected[frame_idx * NUM_CAMERA + cam_idx]) {
					if (exclusion_exists > 0) {
						if (!measurement_excluded[frame_idx*NUM_CAMERA*NUM_POINTS + cam_idx*NUM_POINTS + world_point_idx]) {
							cam_used++;
							cam_used_per_point++;
							ceres::CostFunction* cost_function = ReprojectionError::Create(measurements_x[measurement_idx + NUM_POINTS * cam_idx], measurements_y[measurement_idx + NUM_POINTS * cam_idx], *cam_params[cam_idx]);
							problem.AddResidualBlock(cost_function, loss, world_point);
							
						}
						else {
							excluded_count++;
						}
					}
					else {
						cam_used++;
						cam_used_per_point++;
						ceres::CostFunction* cost_function = ReprojectionError::Create(measurements_x[measurement_idx + NUM_POINTS * cam_idx], measurements_y[measurement_idx + NUM_POINTS * cam_idx], *cam_params[cam_idx]);
						problem.AddResidualBlock(cost_function, loss, world_point);
					}
				}
			}

			if (cam_used_per_point > 1) {
				actual_num_3d_points++;
				// at least 1 camera saw this 3d point
				ceres::Solver::Options options;
				options.linear_solver_type = ceres::DENSE_SCHUR;
				options.minimizer_progress_to_stdout = 0;

				ceres::Solver::Summary summary;
				ceres::Solve(options, &problem, &summary);
				//cout << summary.FullReport() << endl;
				//cout << "Estimated world point = (" << world_point[0] << ", " << world_point[1] << ", " << world_point[2] << ")" << endl;
				world_point_output[world_point_idx][0] = world_point[0];
				world_point_output[world_point_idx][1] = world_point[1];
				world_point_output[world_point_idx][2] = world_point[2];
				final_costs[world_point_idx] = summary.final_cost;
				cost_sum_final += summary.final_cost;
				cost_sum_initial += summary.initial_cost;
			}
			else {
				// less than 2 cameras saw this point -> cannot run triangulation
				world_point_output[world_point_idx][0] = -1;
				world_point_output[world_point_idx][1] = -1;
				world_point_output[world_point_idx][2] = -1;
				final_costs[world_point_idx] = 0;
			}
		} // for each worldpoint ends

		output_txt << triangulation_problem.img_names_[frame_idx] << " ";
		for (int cam_idx = 0; cam_idx < NUM_CAMERA; cam_idx++) {
			output_txt << g_chb_detected[frame_idx * NUM_CAMERA + cam_idx] << " ";
		}

		for (int pi = 0; pi < NUM_POINTS; pi++) {
			output_txt << world_point_output[pi][0] << " " << world_point_output[pi][1] << " " << world_point_output[pi][2] << " ";
		}
		for (int pi = 0; pi < NUM_POINTS; pi++) {
			output_txt << final_costs[pi] << " ";
		}
		output_txt << "\n";
	} // for each frame_idx ends



	output_txt.close();
	printf("Total costs: %f -> %f\n", cost_sum_initial, cost_sum_final);
	float p_percent = float(actual_num_3d_points) / float(total_num_3d_points) * 100.f;
	printf("\n>> Complete. %d 3d points triangulated, out of expected %d points (%.2f%%).\n", actual_num_3d_points, total_num_3d_points, p_percent);
	std::getchar();
	// clean up
	delete[] measurements_x;
	delete[] measurements_y;
	delete[] measurements;
	if (excluded_count > 0) {
		delete[] measurement_excluded;
	}
	for (int c = 0; c < NUM_CAMERA; c++) {
		delete cam_params[c];
	}
	return 0;
}
