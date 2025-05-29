#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Kích thước mong muốn của ảnh
    int width = 40;
    int height = 20;

    // Tạo một ảnh đen hoàn toàn với kích thước 40x20
    cv::Mat binaryImage = cv::Mat::zeros(height, width, CV_8UC1);

    // Màu trắng (255 cho ảnh 8-bit đơn kênh)
    cv::Scalar white = 255;

    // --- Cố gắng vẽ các hình dạng ở kích thước cực nhỏ ---
    // Điều chỉnh các tọa độ và kích thước cho phù hợp với ảnh 40x20
    // Các giá trị này được ước tính và có thể cần tinh chỉnh để đạt được kết quả mong muốn
    // (nhưng như đã nói, chi tiết sẽ rất hạn chế)

    // Phần bên trái (ví dụ: một hình chữ nhật nhỏ)
    cv::rectangle(binaryImage, cv::Point(5, 5), cv::Point(15, 15), white, cv::FILLED);

    // Một chấm nhỏ để đại diện cho "răng cưa" hoặc phần lồi phía trên
    cv::circle(binaryImage, cv::Point(10, 4), 1, white, cv::FILLED);

    // Phần "lỗ" bên trong phần bên trái
    cv::rectangle(binaryImage, cv::Point(7, 8), cv::Point(9, 10), 0, cv::FILLED);

    // Phần kết nối ở giữa
    cv::rectangle(binaryImage, cv::Point(15, 8), cv::Point(25, 10), white, cv::FILLED);

    // Phần bên phải
    cv::rectangle(binaryImage, cv::Point(25, 5), cv::Point(35, 15), white, cv::FILLED);

    // Phần "lỗ" hoặc chi tiết bên trong phần bên phải
    cv::rectangle(binaryImage, cv::Point(30, 8), cv::Point(32, 10), 0, cv::FILLED);
    cv::rectangle(binaryImage, cv::Point(33, 8), cv::Point(35, 10), 0, cv::FILLED);


    // --- Thay đổi kích thước để hiển thị rõ hơn (tùy chọn) ---
    // Vì ảnh 40x20 quá nhỏ để nhìn rõ, chúng ta có thể phóng to nó lên
    cv::Mat resizedImage;
    int scale = 10; // Phóng to 10 lần
    cv::resize(binaryImage, resizedImage, cv::Size(), scale, scale, cv::INTER_NEAREST);
    // INTER_NEAREST giữ các pixel sắc nét, thích hợp cho ảnh nhị phân

    // Hiển thị ảnh đã thay đổi kích thước
    cv::imshow("Binary Image (Scaled for Viewing)", resizedImage);
    cv::waitKey(0);

    // Lưu ảnh gốc 40x20
    cv::imwrite("D:/FresherXavisTech/Image/1.png", binaryImage);
    std::cout << "Image saved as binary_image_40x20" << std::endl;

    return 0;
}