#include <iostream>

#include <Eigen/Core>
#include <opencv2/core.hpp>

int main()
{
    std::cout << "structured-light-3d-reconstruction\n";

    std::cout
        << "Architecture: "
        << sizeof(void*) * 8
        << " bit\n";

    std::cout
        << "OpenCV version: "
        << cv::getVersionString()
        << '\n';

    std::cout
        << "Eigen version: "
        << EIGEN_VERSION_STRING
        << '\n';

    Eigen::Matrix2d eigen_matrix;
    eigen_matrix <<
        1.0, 2.0,
        3.0, 4.0;

    std::cout
        << "\nEigen matrix:\n"
        << eigen_matrix
        << '\n';

    const cv::Mat cv_matrix =
        cv::Mat::eye(2, 2, CV_64F);

    std::cout
        << "\nOpenCV matrix:\n"
        << cv_matrix
        << '\n';

    std::cout
        << "\nEnvironment check passed.\n";

    return 0;
}