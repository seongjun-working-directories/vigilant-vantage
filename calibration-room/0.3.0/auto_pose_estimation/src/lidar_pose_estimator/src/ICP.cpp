#include "lidar_pose_estimator/ICP.hpp"

cv::Mat dock_ICP::drawDistribution(cv::Mat *input, float normalized)
{
	int height = 100;
	int col = (int)input->cols;
	int row = (int)input->rows;

	cv::Mat output(height * row + 10, cv::max(col + 10, height * 2), CV_8UC3, cv::Scalar(255, 255, 255));

	if (col == 0 || row == 0)
		return output;

	float *max_val = new float[row];
	std::memset(max_val, 0, sizeof(float)*row);   // 메모리의 내용(값)을 원하는 크기만큼 특정 값으로 세팅할 수 있는 함수
	float *min_val = new float[row];
	std::memset(min_val, 0, sizeof(float)*row);
	float *normal = new float[row];								//  1 all positive, 0 both, -1 all negative
	std::memset(normal, 0, sizeof(float)*row);
	float *base_height = new float[row];
	std::memset(base_height, 0, sizeof(float)*row);

	for (int col_i = 0; col_i < col; col_i++)
		for (int row_i = 0; row_i < row; row_i++)
		{
			if (input->at<float>(row_i, col_i) > max_val[row_i]) 	// 영상의 픽셀 값에 접근하기 위해 at 함수 사용
			{
				max_val[row_i] = input->at<float>(row_i, col_i);
			}
			if (input->at<float>(row_i, col_i) < min_val[row_i])
			{
				min_val[row_i] = input->at<float>(row_i, col_i);
			}
		}

	for (int row_i = 0; row_i < row; row_i++)
	{
		if (max_val[row_i] == 0 && min_val[row_i] == 0)
		{
			normal[row_i] = 1.0f;
			base_height[row_i] = (float)((row_i + 1)*height);
		}
		else if (max_val[row_i] == 0)
		{
			normal[row_i] = abs(min_val[row_i]);
			base_height[row_i] = (float)((row_i)*height) + 0.1f*(float)height;
		}
		else if (min_val[row_i] == 0)
		{
			normal[row_i] = abs(max_val[row_i]);;
			base_height[row_i] = (float)((row_i + 1)*height);
		}
		else
		{
			normal[row_i] = cv::max(abs(max_val[row_i]), abs(min_val[row_i]));
			base_height[row_i] = (float)((row_i)*height) + 0.5f*(float)height;
		}
	}

	if (normalized != 0)
	{
		for (int row_i = 0; row_i < row; row_i++)
		{
			normal[row_i] = abs(normalized);
		}
	}

	for (int col_i = 0; col_i < col; col_i++)
		for (int row_i = 0; row_i < row; row_i++)
		{
			cv::Point base = cv::Point(col_i, (int)base_height[row_i]);
			cv::Point top = cv::Point(col_i, (int)(base_height[row_i] - 0.9f*(float)height * (input->at<float>(row_i, col_i) / normal[row_i])));
			cv::line(output, base, top, cv::Scalar(0, 0, 0));
			// 직선그리기 line(Input OUtput Array(직선 그려넣을 이미지 입력), pt1,pt2, scalr(0,0,0)-> 직선의 색상(r,g,b 순서))
		}

	return output;

	delete[] max_val;
	delete[] min_val;
	delete[] normal;
	delete[] base_height;
}
bool dock_ICP::getRTMatrixSVD_filter(const cv::Point2f* r, const cv::Point2f* m, int nb_point_ref,  int count, CvMat * r_final, CvMat *t_final, float *inlierratio) 
{

	cv::Mat image = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
	bool find = false;
	float Histo_Ref[360];
	float Histo_new[360];
	float diff_new[360];
	for (int i = 0; i < 360; i++)
	{
		Histo_Ref[i] = 0.0;
		Histo_new[i] = 0.0;
		diff_new[i] = 0.0;
	}


	for (int i = 0; i < nb_point_ref; i++)
	{
		// reference 각도를 구해서 rad->degree로 변환
		int refangle = (int) (atan2((float)r[i].x, (float)r[i].y) * 180 / CV_PI);

		if (refangle < 0) refangle = refangle + 360;

		// 거리 계산(제곱근 계산)
		Histo_Ref[refangle] = sqrt(r[i].x * r[i].x + r[i].y * r[i].y);

		// 행렬에 원을 그려주는 함수-(image:원을 그릴 대상 행렬, 원의 중심 좌표(x,y), 2:반지름, CV_RGB(255, 255, 255): 선의 색상,흰색
		cv::circle(image, cvPoint(250 + (int)r[i].x * (50 / 1000.0), 250 - (int)r[i].y * (50 / 1000.0)), 2, CV_RGB(255, 255, 255), 1);
	}

	for (int i = 0; i < count; i++)
	{
		int newangle = (int)(atan2((float)m[i].x, (float)m[i].y) * 180 / CV_PI);
		if (newangle < 0) newangle = newangle + 360;
		Histo_new[newangle] = sqrt(m[i].x * m[i].x + m[i].y * m[i].y);
	}
	std::vector<cv::Point2f> New_points;
	int filteredpoint = 0;
	int rejectedpoint = 0;
	for (int i = 0; i < 360; i++)
	{
		if (Histo_Ref[i] != 0.0 && Histo_new[i] != 0.0)
		{
			diff_new[i] = (abs)((Histo_Ref[i] - Histo_new[i]));

			if (diff_new[i] < 150)
			{
				float dist = Histo_new[i];
				float x = dist * sin(i *  CV_PI / 180.0);
				float y = dist * cos(i *  CV_PI / 180.0);
				New_points.push_back(cv::Point2f(x, y));
				cv::circle(image, cvPoint(250 + (int)x * (50 / 1000.0), 250 - (int)y * (50 / 1000.0)), 5, CV_RGB(0, 255, 0), 1);
				filteredpoint++;
			}
			else
			{
				float dist = Histo_new[i];
				float x = dist * sin(i *  CV_PI / 180.0);
				float y = dist * cos(i *  CV_PI / 180.0);
				cv::circle(image, cvPoint(250 + (int)x * (50 / 1000.0), 250 - (int)y * (50 / 1000.0)), 1, CV_RGB(255, 0, 0), 1);
				rejectedpoint++;
			}
		}
	}
	float inlier = (float)filteredpoint / (float)(filteredpoint + rejectedpoint);
	*inlierratio = inlier;

	if (inlier > 0.6)
	{
		int k, i;
		float err;
		float prev_err = FLT_MAX;
		struct kdtree *ptree = kd_create(2);
		cv::Point2f * input_correlation_old = (cv::Point2f *)malloc(sizeof(cv::Point2f)*filteredpoint);
		cv::Point2f * input_correlation_new = (cv::Point2f *)malloc(sizeof(cv::Point2f)*filteredpoint);

		for (i = 0; i < nb_point_ref; i++)
			kd_insertf((struct kdtree*) ptree, (float*)&r[i], 0);

		for (i = 0; i < filteredpoint; i++)
			input_correlation_new[i] = New_points[i];

		float R[4]; CvMat r = cvMat(2, 2, CV_32F, R);
		float T[2]; CvMat t = cvMat(2, 1, CV_32F, T);

		for (k = 0; k < 10; k++) {
			float R[4]; CvMat r = cvMat(2, 2, CV_32F, R);
			float T[2]; CvMat t = cvMat(2, 1, CV_32F, T);

			err = 0.;
			for (i = 0; i < filteredpoint; i++) {
				struct kdres * presults = kd_nearestf((struct kdtree *)ptree, (float*)&input_correlation_new[i]);
				kd_res_end(presults);
				kd_res_itemf(presults, (float*)&input_correlation_old[i]);
				err += sqrtf(dist_sq((float*)&input_correlation_old[i], (float*)&input_correlation_new[i], 2));

				kd_res_free(presults);
			}
			getRTMatrixSVD(&input_correlation_new[0], &input_correlation_old[0], filteredpoint, &r, &t);
			for (i = 0; i < filteredpoint; i++) {
				float x = input_correlation_new[i].x;
				float y = input_correlation_new[i].y;
				float X = (R[0] * x + R[1] * y + T[0]);
				float Y = (R[2] * x + R[3] * y + T[1]);

				input_correlation_new[i].x = X;
				input_correlation_new[i].y = Y;

			}
			if (fabs(err - prev_err) < 0.1)  break;
			else prev_err = err;
		}

		getRTMatrixSVD(&New_points[0], &input_correlation_new[0], filteredpoint, r_final, t_final);

		kd_free(ptree);
		free(input_correlation_old);
		free(input_correlation_new);
		find = true;
	}

	cv::Mat plot_score(1, 360, CV_32FC1, diff_new);
	cv::Mat plot_score1(1, 360, CV_32FC1, Histo_Ref);
	cv::Mat plot_score2(1, 360, CV_32FC1, Histo_new);

	cv::imshow("ref1 & current points", image);
	cv::waitKey(10);

	std::vector<uchar> data;
  sensor_msgs::CompressedImage compressed_image;
  compressed_image.header.stamp = ros::Time::now();
  compressed_image.format = "jpeg";
  cv::imencode(".jpg", image, data);
  compressed_image.data = data;
  compressed_image_publisher_1.publish(compressed_image);
	New_points.clear();

	return find;

}
bool dock_ICP::getCovICP(const cv::Point2f* r, int nb_point_ref, const cv::Point2f* m, int nb_ref,  float *dxx, float *dxy, float *dyx, float *dyy)
{
	int k, i;
	float err = 0.0;
	float err_x = 0.0;
	float err_y = 0.0;
	float prev_err = FLT_MAX;
	struct kdtree *ptree = kd_create(2);
	cv::Point2f * input_correlation_old = (cv::Point2f *)malloc(sizeof(cv::Point2f)*nb_point_ref);
	cv::Point2f * input_correlation_new = (cv::Point2f *)malloc(sizeof(cv::Point2f)*nb_point_ref);

	for (i = 0; i < nb_ref; i++)
		kd_insertf((struct kdtree*) ptree, (float*)&m[i], 0);

	for (i = 0; i < nb_point_ref; i++)
	{
		input_correlation_new[i].x = r[i].x + 200;
		input_correlation_new[i].y = r[i].y;
	}

	for (i = 0; i < nb_point_ref; i++)
	{
		struct kdres * presults = kd_nearestf((struct kdtree *)ptree, (float*)&input_correlation_new[i]);
		kd_res_end(presults);
		kd_res_itemf(presults, (float*)&input_correlation_old[i]);
		err += sqrtf(dist_sq((float*)&input_correlation_old[i], (float*)&input_correlation_new[i], 2));

		err_x += ((input_correlation_new[i].x - input_correlation_old[i].x)) / 200.0;
		err_y += ((input_correlation_new[i].y - input_correlation_old[i].y)) / 200.0;


		kd_res_free(presults);
	}

	*dxx = (err_x / nb_point_ref) * 1000;
	*dxy = -(err_y/ nb_point_ref) * 1000;

	float err_x1 = 0.0;
	float err_y1 = 0.0;

	for (i = 0; i < nb_point_ref; i++)
	{
		input_correlation_new[i].x = r[i].x;
		input_correlation_new[i].y = r[i].y + 200;
	}

	for (i = 0; i < nb_point_ref; i++)
	{
		struct kdres * presults = kd_nearestf((struct kdtree *)ptree, (float*)&input_correlation_new[i]);
		kd_res_end(presults);
		kd_res_itemf(presults, (float*)&input_correlation_old[i]);
		err += sqrtf(dist_sq((float*)&input_correlation_old[i], (float*)&input_correlation_new[i], 2));

		err_x1 += ((input_correlation_new[i].x - input_correlation_old[i].x)) / 200.0;
		err_y1 += ((input_correlation_new[i].y - input_correlation_old[i].y)) / 200.0;


		kd_res_free(presults);
	}

	*dyx = -(err_x1 / nb_point_ref) * 1000;
	*dyy = (err_y1 / nb_point_ref)* 1000;


	kd_free(ptree);
	free(input_correlation_old);
	free(input_correlation_new);


	return true;
}

void dock_ICP::getRTMatrixSVD(const cv::Point2f* a, const cv::Point2f* b, int count, CvMat *r, CvMat *t)
{
	int i;
	float H[4] = { 0.f,0.f,0.f,0.f }; CvMat h = cvMat(2, 2, CV_32F, H);
	float U[4] = { 0.f,0.f,0.f,0.f }; CvMat u = cvMat(2, 2, CV_32F, U);
	float W[4] = { 0.f,0.f,0.f,0.f }; CvMat w = cvMat(2, 2, CV_32F, W);
	float V[4] = { 0.f,0.f,0.f,0.f }; CvMat v = cvMat(2, 2, CV_32F, V);
	cv::Point2f mean_a = cv::Point2f(0.f, 0.f);
	cv::Point2f mean_b = cv::Point2f(0.f, 0.f);

	for (i = 0; i < count; i++) {
		mean_a.x += a[i].x;
		mean_a.y += a[i].y;
		mean_b.x += b[i].x;
		mean_b.y += b[i].y;
	}
	mean_a.x /= (float)count;
	mean_a.y /= (float)count;
	mean_b.x /= (float)count;
	mean_b.y /= (float)count;
	for (i = 0; i < count; i++) {
		float AX = (a[i].x - mean_a.x);
		float AY = (a[i].y - mean_a.y);
		float BX = (b[i].x - mean_b.x);
		float BY = (b[i].y - mean_b.y);
		H[0] += AX*BX;
		H[1] += AX*BY;
		H[2] += AY*BX;
		H[3] += AY*BY;
	}

	cvSVD(&h, &w, &u, &v, CV_SVD_MODIFY_A);

	{
		// get result :
		float * R = r->data.fl;
		float * T = t->data.fl;
		// R = V * transpose(U)
		R[0] = V[0] * U[0] + V[1] * U[1];
		R[1] = V[0] * U[2] + V[1] * U[3];
		R[2] = V[2] * U[0] + V[3] * U[1];
		R[3] = V[2] * U[2] + V[3] * U[3];

		// special reflection case
		if (cvDet(r) < 0) {
			R[0] = V[0] * U[0] - V[1] * U[1];
			R[1] = V[0] * U[2] - V[1] * U[3];
			R[2] = V[2] * U[0] - V[3] * U[1];
			R[3] = V[2] * U[2] - V[3] * U[3];
		}

		// T = -R*meanA+meanB
		T[0] = -R[0] * mean_a.x - R[1] * mean_a.y + mean_b.x;
		T[1] = -R[2] * mean_a.x - R[3] * mean_a.y + mean_b.y;
	}
}

/* returns the distance squared between two dims-dimensional double arrays */
float dock_ICP::dist_sq(float *a1, float*a2, int dims) {
	float dist_sq = 0, diff;
	while (--dims >= 0) {
		diff = (a1[dims] - a2[dims]);
		dist_sq += diff * diff;
	}
	return dist_sq;
}

float dock_ICP::icp(const cv::Point2f* new_points, int nb_point_new, const cv::Point2f* ref_points, int nb_point_ref, CvMat * r_final, CvMat *t_final, CvTermCriteria criteria )
{
	cv::Mat image = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
	int k, i;
	float prev_err = FLT_MAX;
	float err;
	struct kdtree *ptree = kd_create(2);
	cv::Point2f * input_correlation_old = (cv::Point2f *)malloc(sizeof(cv::Point2f)*nb_point_new);
	cv::Point2f * input_correlation_new = (cv::Point2f *)malloc(sizeof(cv::Point2f)*nb_point_new);


	r_final->data.fl[0] = 1.f; r_final->data.fl[1] = 0.f;
	r_final->data.fl[2] = 0.f; r_final->data.fl[3] = 1.f;
	t_final->data.fl[0] = 0.f;
	t_final->data.fl[1] = 0.f;

	for (i = 0; i < nb_point_ref; i++)
	{
		kd_insertf((struct kdtree*) ptree, (float*)&ref_points[i], 0);
	}

	for (i = 0; i < nb_point_new; i++)
		input_correlation_new[i] = new_points[i];

	for (k = 0; k < criteria.max_iter; k++)
	{
		image = cv::Mat::zeros(500, 500, CV_8UC3 );
		float R[4];
		CvMat r = cvMat(2, 2, CV_32F, R);
		float T[2];
		CvMat t = cvMat(2, 1, CV_32F, T);

		err = 0.;
		for (i = 0; i < nb_point_new; i++)
		{
			struct kdres *presults = kd_nearestf((struct kdtree *)ptree, (float *)&input_correlation_new[i]);
			kd_res_end(presults);
			kd_res_itemf(presults, (float *)&input_correlation_old[i]);
			err += sqrtf(dist_sq((float *)&input_correlation_old[i], (float *)&input_correlation_new[i], 2));

			if (!image.empty())
			{
				cv::circle(image, cvPoint(250 + (int)input_correlation_new[i].x * (100 / 1000.0), 250 - (int)input_correlation_new[i].y * (100 / 1000.0)), 1, CV_RGB(255, 0, 0), 1);
				cv::circle(image, cvPoint(250 + (int)input_correlation_old[i].x * (100 / 1000.0), 250 - (int)input_correlation_old[i].y * (100 / 1000.0)), 3, CV_RGB(0, 0, 255), 1);				
				cv::line(image, cvPoint(250 + (int)input_correlation_old[i].x * (100 / 1000.0), 250 - (int)input_correlation_old[i].y * (100 / 1000.0)), cvPoint(250 + (int)input_correlation_new[i].x * (100 / 1000.0), 250 - (int)input_correlation_new[i].y * (100 / 1000.0)), CV_RGB(0, 255, 0), 1, 8, 0);				
			}

			kd_res_free(presults);
		}
		// std::cout << "iter: "<< k << " err: "<< err <<std::endl;
		//cv::imshow("image test", image);
		cv::imshow("ICP processing", image);
		cv::waitKey(10);

		std::vector<uchar> data;
    sensor_msgs::CompressedImage compressed_image_2;
    compressed_image_2.header.stamp = ros::Time::now();
    compressed_image_2.format = "jpeg";
    cv::imencode(".jpg", image, data);
    compressed_image_2.data = data;
    compressed_image_publisher_2.publish(compressed_image_2);

		getRTMatrixSVD(&input_correlation_new[0], &input_correlation_old[0], nb_point_new, &r, &t);
		for (i = 0; i < nb_point_new; i++) {
			float x = input_correlation_new[i].x;
			float y = input_correlation_new[i].y;
			float X = (R[0] * x + R[1] * y + T[0]);
			float Y = (R[2] * x + R[3] * y + T[1]);

			input_correlation_new[i].x = X;
			input_correlation_new[i].y = Y;

		}
		if (fabs(err - prev_err) < criteria.epsilon)  break;
		else prev_err = err;
	}

	err = err / nb_point_new;

	float R1[4] = { 1.f,0.f,0.f,1.f }, T1[2] = { 0.,0. };
	CvMat r1 = cvMat(2, 2, CV_32F, R1);
	CvMat t1 = cvMat(2, 1, CV_32F, T1);

	getRTMatrixSVD(&new_points[0], &input_correlation_new[0], nb_point_new, &r1, &t1);
	// printf("getRTMatrixSVD ICP transformation : \n");
	// printf("R[0]=%f R[1]=%f T[0]=%f\n", r1.data.fl[0], r1.data.fl[1], t1.data.fl[0]);
	// printf("R[2]=%f R[3]=%f T[1]=%f\n\n", r1.data.fl[2], r1.data.fl[3], t1.data.fl[1]);


	float R2[4] = { 1.f,0.f,0.f,1.f }, T2[2] = { 0.,0. };
	CvMat r2 = cvMat(2, 2, CV_32F, R2);
	CvMat t2 = cvMat(2, 1, CV_32F, T2);

	bool find = false;
	find = getRTMatrixSVD_filter(&ref_points[0], &input_correlation_new[0], nb_point_ref, nb_point_new, &r2, &t2, &inlierratio);

	float test_R1[4] = { 1.f,0.f,0.f,1.f }, test_T1[2] = { 0.,0. };
	test_r1 = cvMat(2, 2, CV_32F, test_R1);
	test_t1 = cvMat(2, 1, CV_32F, test_T1);



	if(find){
		getRTMatrixSVD(&input_correlation_new[0], &new_points[0], nb_point_new, &test_r1, &test_t1);
		// printf("filter ICP transformation : \n");
		// printf("R[0]=%f R[1]=%f T[0]=%f\n", r2.data.fl[0], r2.data.fl[1], t2.data.fl[0]);
		// printf("R[2]=%f R[3]=%f T[1]=%f\n\n", r2.data.fl[2], r2.data.fl[3], t2.data.fl[1]);
	}
	else
	{
		err = -1;
	}

	r_final->data.fl[0] = (r1.data.fl[0] * r2.data.fl[0] + r1.data.fl[1] * r2.data.fl[2]);
	r_final->data.fl[1] = (r1.data.fl[0] * r2.data.fl[1] + r1.data.fl[1] * r2.data.fl[3]);
	r_final->data.fl[2] = (r1.data.fl[2] * r2.data.fl[0] + r1.data.fl[3] * r2.data.fl[2]);
	r_final->data.fl[3] = (r1.data.fl[2] * r2.data.fl[1] + r1.data.fl[3] * r2.data.fl[3]);

	t_final->data.fl[0] = (t1.data.fl[0] + t2.data.fl[0]);
	t_final->data.fl[1] = (t1.data.fl[1] + t2.data.fl[1]);
	// printf("filter ICP transformation : \n");
	// printf("R[0]=%f R[1]=%f T[0]=%f\n", r_final->data.fl[0], r_final->data.fl[1], t_final->data.fl[0]);
	// printf("R[2]=%f R[3]=%f T[1]=%f\n\n", r_final->data.fl[2], r_final->data.fl[3], t_final->data.fl[1]);

	RT_ref_data.clear();
	for(int i = 0; i < nb_point_ref; i++){
		cv::Point2f r_p;
		r_p.x = (test_t1.data.fl[0]) + (ref_points[i].x*(test_r1.data.fl[0]) + ref_points[i].y*(test_r1.data.fl[1]));
		r_p.y = (test_t1.data.fl[1]) + (ref_points[i].x*(test_r1.data.fl[2]) + ref_points[i].y*(test_r1.data.fl[3]));
		RT_ref_data.push_back(r_p);
	}

	kd_free(ptree);
	free(input_correlation_old);
	free(input_correlation_new);


	printf("R info\n");
	std::cout <<"test_r1.data.fl[0]:" << test_r1.data.fl[0]<<std::endl;
	std::cout <<"test_r1.data.fl[1]:" << test_r1.data.fl[1]<<std::endl;
	std::cout <<"test_r1.data.fl[2]:" << test_r1.data.fl[2]<<std::endl;
	std::cout <<"test_r1.data.fl[3]:" << test_r1.data.fl[3]<<std::endl;

	float angle = asin(test_r1.data.fl[2]);
	float degree = angle * (180 / M_PI);
	std::cout << "degree = " << degree << std::endl;

	std::cout <<"T info" <<std::endl;
	std::cout<<std::fixed;
	std::cout.precision(5);
	std::cout <<"test_t1.data.fl[0]:" << test_t1.data.fl[0]*0.001 <<std::endl;
	std::cout <<"test_r1.data.fl[1]:" << test_t1.data.fl[1]*0.001<< "\n" <<std::endl;
	std::cout.unsetf(std::ios::fixed);

	float T_x = test_t1.data.fl[0];
	float T_y = test_t1.data.fl[1];
	std::cout << "T:" << T_x <<" , " << T_y << std::endl;

	static tf::TransformBroadcaster tf_broadcaster;
  tf::Transform tf_transform;
  tf::Quaternion tf_quaternion;

  tf_transform.setOrigin(tf::Vector3(T_x, T_y, 0));
	tf_quaternion.setRPY(0, 0, angle);	// radian
  tf_transform.setRotation(tf_quaternion);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "base_link", "lidar_tf"));

	return err;
}