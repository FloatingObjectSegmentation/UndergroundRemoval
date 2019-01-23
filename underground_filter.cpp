// made by mirceta

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <string>
#include <map>
#include <Windows.h>

using namespace std;

#pragma region [configuration]
string dmr_directory = "E:\\workspaces\\LIDAR_WORKSPACE\\test\\";
string dmr_file_name = "dmr";
string aug_directory = "E:\\workspaces\\LIDAR_WORKSPACE\\test\\";
string aug_file_name = "augs";
#pragma endregion

#pragma region [auxiliary]
class AugmentablesFile {
public:
	std::vector<pcl::PointXYZ> points;
	std::vector<int> centerPointIndices;
	std::map<int, int> CentralPointMap;
	std::vector<float> radii;
};

class DMRStruct {
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr pdmr; // DMR point cloud projected to the 3D plane z=0
	std::vector<float> z_vals; // values for z
};

int ReadDMRStructure(DMRStruct& dmr, const string& filepath) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;

	vector<float> z_vals;
	for (auto& x : cloud->points) {
		z_vals.push_back(x.z);
		x.z = 0.0f;
	}

	dmr.pdmr = cloud;
	dmr.z_vals = z_vals;
	
	return 0;
}

int ReadAugmentablesFile(AugmentablesFile &f, const string& filepath) {
	// read the input file <point maxdim> => 1.0,2.0,3.0 5.0
	std::vector<pcl::PointXYZ> points;
	std::vector<int> centerPointIndices;
	std::map<int, int> CentralPointMap; // accepts index of any point and returns the central point index
	std::vector<float> radii;

	ifstream infile;
	infile.open(filepath, ios::in);
	string line;
	int currIdx = 0;
	while (getline(infile, line)) {

		// get the max dimension
		std::vector<std::string> results;
		boost::split(results, line, [](char c) {return c == ' '; });
		float rbnn_r = atof(results[9].c_str());
		radii.push_back(rbnn_r);

		// get the central point
		std::vector<std::string> centralPointStrings;
		boost::split(centralPointStrings, results[0], [](char c) {return c == ','; });
		float x = atof(centralPointStrings[0].c_str());
		float y = atof(centralPointStrings[1].c_str());
		float z = atof(centralPointStrings[2].c_str());
		points.push_back(pcl::PointXYZ(x, y, z));
		int currentCentralPointIndex = currIdx++;
		CentralPointMap[currentCentralPointIndex] = currentCentralPointIndex;
		centerPointIndices.push_back(currentCentralPointIndex);

		// get the boundary points
		for (int i = 1; i < 9; i++) {
			std::vector<std::string> boundaryPointStrings;
			boost::split(boundaryPointStrings, results[i], [](char c) {return c == ','; });
			float x = atof(boundaryPointStrings[0].c_str());
			float y = atof(boundaryPointStrings[1].c_str());
			float z = atof(boundaryPointStrings[2].c_str());
			points.push_back(pcl::PointXYZ(x, y, z));
			CentralPointMap[currIdx++] = currentCentralPointIndex;
		}
	}

	AugmentablesFile retval;
	retval.centerPointIndices = centerPointIndices;
	retval.CentralPointMap = CentralPointMap;
	retval.points = points;
	retval.radii = radii;
	f = retval;

	return 1;
}

int ParseArguments(int argc, char** argv) {
	// parsing arguments
	if (argc == 5) {
		dmr_directory = argv[1];
		dmr_file_name = argv[2];
		aug_directory = argv[3];
		aug_file_name = argv[4];
	}
	else {
		return false;
	}
	return true;
}
#pragma endregion


/*
dmr should be in pcd format.
augmentables should be in the same format as overlap_compute tool's
args: dmr_dir dmr_filename augmentables_dir augmentables_filename

assumptions:
*/
int main (int argc, char** argv)
{

	if (!ParseArguments(argc, argv)) {
		return 1;
	}

	// read inputs
	AugmentablesFile augs;
	ReadAugmentablesFile(augs, aug_directory + "\\" + aug_file_name);
	DMRStruct dmr;
	ReadDMRStructure(dmr, dmr_directory + "\\" + dmr_file_name);
	

	float dmr_density = max(abs(dmr.pdmr->points[0].x - dmr.pdmr->points[1].x), 
							abs(dmr.pdmr->points[0].y - dmr.pdmr->points[1].y));

	// init DMR KDTree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(dmr.pdmr);

	std::set<int> underground;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	for (int i = 0; i < augs.points.size(); i++) 
	{

		pcl::PointXYZ pp(augs.points[i].x, augs.points[i].y, 0.0f); // projected point
		float p_z = augs.points[i].z;

		if (kdtree.radiusSearch(pp, sqrt(2 * pow(dmr_density, 2)) + 0.1f, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			if (pointIdxRadiusSearch.empty())
			{
				PCL_ERROR("At least one of the augmentations is not defined in the area of the DMR at all!\n");
			}
			else 
			{
				// find the closest point
				float min_r = 10.0f;
				int min_r_idx = 0;
				//std::cout << pp.x << " " << pp.y << std::endl;
				for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
					int idx = pointIdxRadiusSearch[j];
					pcl::PointXYZ nbr = dmr.pdmr->points[idx];
					//std::cout << nbr.x << " " << nbr.y << std::endl;
					float dist = sqrt(pow(nbr.x - pp.x, 2) + pow(nbr.y - pp.y, 2));
					if (dist < min_r) {
						min_r = dist;
						min_r_idx = j;
					}
				}

				// if the closest point is higher, then filter, if not then don't filter
				if (dmr.z_vals[min_r_idx] > p_z)
					underground.insert(augs.CentralPointMap[i] / 9); // because there are 9 points per bounding box that stick together.
			}
		}
	}

	// write to the output file
	ofstream outfile;
	outfile.open(aug_directory + "\\" + "underground" + aug_file_name, ios::out);
	stringstream ss;
	for (auto & i : underground) {
		ss << i << " ";
	}
	outfile << ss.str().substr(0, ss.str().length() - 1);
	outfile.close();
	cout << "results written to disc. Press any key to exit..." << endl;
	
	return 0;
}