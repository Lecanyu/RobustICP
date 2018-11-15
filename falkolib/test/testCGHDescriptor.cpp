//
// Created by lecanyu on 18-11-15.
//

#include <iostream>

#include <falkolib/Common/LaserScan.h>
#include <falkolib/Feature/FALKOExtractor.h>
#include <falkolib/Feature/CGHExtractor.h>

#include "testData.h"

using namespace std;
using namespace falkolib;

int main(int argc, char** argv) {
    LaserScan scan1(-0.003316126, 2.0 * M_PI, 1440);
    scan1.fromRanges(testRanges);
    LaserScan scan2(-0.003316126, 2.0 * M_PI, 1440);
    scan2.fromRanges(testRanges2);

    FALKOExtractor fe;
    fe.setMinExtractionRange(0);
    fe.setMaxExtractionRange(30);
    fe.enableSubbeam(true);
    fe.setNMSRadius(0.1);
    fe.setNeighB(0.07);
    fe.setBRatio(2.5);
    fe.setGridSectors(16);

    std::vector<FALKO> keypoints1;
    std::vector<FALKO> keypoints2;

    fe.extract(scan1, keypoints1);
    fe.extract(scan2, keypoints2);

    cout << "num keypoints1 extracted: " << keypoints1.size() << endl;
    cout << "num keypoints2 extracted: " << keypoints2.size() << endl;

    CGHExtractor<FALKO> cgh(16);
    vector<CGH> cghDesc1;
    vector<CGH> cghDesc2;

    cgh.compute(scan1, keypoints1, cghDesc1);
    cgh.compute(scan2, keypoints2, cghDesc2);

    // output point position and descriptor vector
    for(int i=0; i<keypoints1.size(); ++i)
    {
        double x = scan1.points[keypoints1[i].index](0);
        double y = scan1.points[keypoints1[i].index](1);
        auto descriptor_vec = cghDesc1[i].getDiscriptor();

        std::cout<<"descriptor length: "<<descriptor_vec.size()<<"\n";

        std::cout<<"point: "<<x<<", "<<y<<"\n";
        std::cout<<"descriptor: ";
        for(auto v:descriptor_vec)
            std::cout<< v<<" ";
        std::cout<<"\n\n";
    }


    return 0;
}