#include <string>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


bool get_filelist_from_dir(std::string _path, std::vector<std::string>& _files)
{
    DIR* dir = opendir(_path.c_str());
    struct dirent* ptr;
    std::vector<std::string> file;
    while((ptr = readdir(dir)) != NULL)
    {
        if(ptr->d_name[0] == '.')	continue;
        file.push_back(ptr->d_name);
    }
    closedir(dir);
    sort(file.begin(), file.end());
    _files = file;
}

void convertPCDtoBin(std::string & in_file, std::string & out_file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1)
    {
        std::string err = "Couldn't read file " + in_file;
        PCL_ERROR(err.c_str());
        return;
    }
    std::cout << "Loaded "
         << cloud->width * cloud->height
         << " data points from "
         << in_file
         << " with the following fields: "
         << std::endl;

    std::ofstream output(out_file.c_str(), std::ios::out | std::ios::binary);
    float intensity = 1.0;
    for (int j = 0; j < cloud->size(); j++)
    {
        output.write((char*)& cloud->at(j).x, sizeof(cloud->at(j).x));
        output.write((char*)& cloud->at(j).y, sizeof(cloud->at(j).y));
        output.write((char*)& cloud->at(j).z, sizeof(cloud->at(j).z));
        intensity = cloud->at(j).intensity / 255.0;
        output.write((char*)& intensity, sizeof(intensity));
        /*std::cout << cloud->at(j).x << " "
                  << cloud->at(j).y << " "
                  << cloud->at(j).z << " "
                  << intensity << std::endl;*/
    }
    output.close();
}

int fileNum =20000;


int main(){
    
    char abs_path[PATH_MAX];
    realpath(__FILE__, abs_path);
    std::string dirpath(abs_path);
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    dirpath = dirpath.substr(0, dirpath.find_last_of("/"));
    std::string pcd_dir = dirpath + "/data/Scans/";
    std::string bin_dir = dirpath + "/data/bin/";

    std::vector<std::string> pcdNames;
    pcdNames.reserve(fileNum);
    get_filelist_from_dir(pcd_dir, pcdNames);

    for(int i = 0; i < pcdNames.size(); i++) {
        std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<< " << i << " >>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        std::string pcdfile = pcd_dir + pcdNames[i].c_str();
        std::string frameName =  pcdNames[i].substr(0,pcdNames[i].find('.',0));
        std::string binfile = bin_dir + frameName +".bin";
        convertPCDtoBin(pcdfile, binfile);

    }
}

