//
//	Algorithms Analysis - Texture Transfer v2.1
//	- main.cpp
//
//	Created by ShauShian, Chiang on 2017/06/06.
//	Copyright (c) 2017 ShauShian, Chiang. All rights reserved.


#include "opencv2/imgproc/imgproc.hpp" // In order to use cvtColor()
#include "texture_synthesis.h" // In order to synthesis patches

#define DEBUG(x) do{  cerr << x << endl;  } while(0)

int get_texture_image_diff(Point texture, Point image, Size patch_size);
Mat find_similiar_from_texture(Mat texture, Point part_img, Size patch_size);
Mat texture_transfer(Mat texture, Mat image, Size patch_size, int overlap);
Mat get_a_row_patch(Mat texture, Mat image, Point coordinate, Size patch_size, int overlap);

static string tab;
Mat texture_gs, image_gs;

int main(int argc, char* argv[]) {

	char* infilename = get_cmd_option(argv, argv + argc, "-i");
	char* outfilename = get_cmd_option(argv, argv + argc, "-o");
	char* texturename = get_cmd_option(argv, argv + argc, "-t");
	char* patchsize = get_cmd_option(argv, argv + argc, "-p");
	char* overlap_str = get_cmd_option(argv, argv + argc, "-r");

	if (infilename && outfilename && texturename && patchsize && overlap_str) {

		Mat texture = imread(texturename, CV_LOAD_IMAGE_COLOR);
		Mat image = imread(infilename, CV_LOAD_IMAGE_COLOR);
		if (texture.data && image.data) {
			int overlap = atoi(overlap_str);
			int patch = atoi(patchsize);

			cvtColor(texture, texture_gs, CV_BGR2GRAY); // Turn to gray scale
			cvtColor(image, image_gs, CV_BGR2GRAY); // Turn to gray scale

			Mat total = texture_transfer(texture, image, Size(patch, patch), overlap);

			namedWindow("Display window", WINDOW_AUTOSIZE);
			imshow("Display window", total);
			imwrite(outfilename, total);

			waitKey(0);
		}
		else
			cout << "Could not open texture or image file...\n";
	}
	else if (cmd_exist(argv, argv + argc, "-h")) {
		cout << "-i Input filename\n" << "-o Output filename\n" << "-t Texture filename\n" << "-t Patch size\n" << "-r Pixel of overlap\n";
	}
	else
		cout << "Wrong command option... try -h option\n";

	return 0;
}

Mat texture_transfer(Mat texture, Mat image, Size patch_size, int overlap) {
	// Use get_a_row_patch function to generate a row of image that is synthesised,
	// and vertically synthesis evey row of image.

	Mat patch_row, total_img; // patch row is produced by get_a_row_patch(), total_img is the sum up of previous row.

	Size img_size = image.size();
	Size tmp_size = patch_size;

	Point coordinate = Point(0, 0);

	total_img = get_a_row_patch(texture, image, coordinate, patch_size, overlap); // We can get a row of synthesised image.
	coordinate.y += patch_size.height - overlap;

	while (true) {
		patch_row = get_a_row_patch(texture, image, coordinate, patch_size, overlap);
		coordinate.y += patch_size.height - overlap;
		total_img = synthesis_two_image_vertical(total_img, patch_row, overlap);

		if (coordinate.y + patch_size.height >= img_size.height) { // It is at the edge of the image.
			tmp_size.height = img_size.height - total_img.size().height + overlap;
			patch_row = get_a_row_patch(texture, image, coordinate, tmp_size, overlap);

			total_img = synthesis_two_image_vertical(total_img, patch_row, overlap);
			break;
		}
	}

	cout << "Texture transfer... Done\n";

	return total_img;
}

Mat get_a_row_patch(Mat texture, Mat image, Point coordinate, Size patch_size, int overlap) {
	// Use the coordinate point to horizontally synthesis a patch set in a row.

	Size img_size = image.size();
	Size temp_size = patch_size; // This size can only be used to fit the edge of the image.

	Mat patch_img; // The image that is on patching.

	Mat sum_img = find_similiar_from_texture(texture, coordinate, patch_size); //
	coordinate.x += patch_size.width - overlap; // Moving to the next coordinate by adding a overlap.

	while (true) {
		patch_img = find_similiar_from_texture(texture, coordinate, patch_size);
		coordinate.x += patch_size.width - overlap;
		sum_img = synthesis_two_image_horizontal(sum_img, patch_img, overlap);

		if (coordinate.x + patch_size.width >= img_size.width) { // Will the next patch image exceed the total image ?
			temp_size.width = img_size.width - sum_img.size().width + overlap; // Change the patch size to meet the edge.
			patch_img = find_similiar_from_texture(texture, coordinate, temp_size); // Find the patch of texture.

			sum_img = synthesis_two_image_horizontal(sum_img, patch_img, overlap); // Adding up these two images.
			break;
		}
	}

	cout << "Coordinate Y : " << coordinate.y << " is completed...\n";
	return sum_img;
}

int get_texture_image_diff(Point texture, Point image, Size patch_size) {
	// Input a texture and an image with the same patch size,
	// and evaluate the gray scale by summing every absolute pixel difference up.
	int t_pixel = 0, i_pixel = 0, sum = 0;
	for (int i = texture.x, m = image.x ; i < texture.x + patch_size.width; i++, m++) {
		for (int j = texture.y, n = image.y ; j < texture.y + patch_size.height; j++, n++) {
			t_pixel = (int)texture_gs.at<uchar>(j, i);
			i_pixel = (int)image_gs.at<uchar>(n, m);

			sum += abs(t_pixel - i_pixel);
		}
	}

	return sum;
}

Mat find_similiar_from_texture(Mat texture, Point part_img, Size patch_size) {
	// Giving a texture and a patch size of image, and finding the most similiar part to the image from texture.
	Size t_size = texture.size();
	int diff = 0;

	Point min_dff_cor; // The coordinate of smallest diff.
	int min_diff = -1; // The value of smallest diff.

	for (int y = 0; y + patch_size.height < t_size.height; y++)
		for (int x = 0; x + patch_size.width < t_size.width; x++) {
			diff = get_texture_image_diff(Point(x,y), part_img, patch_size); // Caculate its grayscale diff.

			if (min_diff == -1 || min_diff > diff) { // If there is a smaller one.
				min_diff = diff;
				min_dff_cor = Point(x, y);
				
			}
		}

	//	cout << "(x,y) = " << min_dff_cor.x << ", " << min_dff_cor.y << endl;
	return texture(Rect(min_dff_cor, patch_size)); // Return the similiar part of image.
}
