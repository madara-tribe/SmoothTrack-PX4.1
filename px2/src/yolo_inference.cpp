#include "yolo_inference.h"
#define ACCURACY 0.25
#define WARNING false

std::vector<std::string> classNames = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "potted plant", "bed", "dining table", "toilet", "tv monitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

std::string TARGET = "clock";

YoloDetect::YoloDetect(const std::string& modelPath){
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    sessionOptions.SetIntraOpNumThreads(numthreads);
    if (!WARNING){
        sessionOptions.SetLogSeverityLevel(4);  // erase waring message
    }
    session = new Ort::Session(env, modelPath.c_str(), sessionOptions);
}

std::vector<Result> YoloDetect::postprocess(cv::Size originalImageSize, std::vector<Ort::Value>& outputTensors)
{
    auto* rawOutput = outputTensors[0].GetTensorData<float>();
    std::vector<int64_t> outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
    size_t count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    std::vector<float> output(rawOutput, rawOutput + count);

    std::vector<Result> resultVector;

    for (int i = 0; i < outputShape[0]; i++) {

        float confidence        = output[i * outputShape[1] + 0];
        float x1                = output[i * outputShape[1] + 1];
        float y1                = output[i * outputShape[1] + 2];
        float x2                = output[i * outputShape[1] + 3];
        float y2                = output[i * outputShape[1] + 4];
        int classPrediction     = output[i * outputShape[1] + 5];
        float accuracy          = output[i * outputShape[1] + 6];

        (void) confidence;
        if (classNames.at(classPrediction) == TARGET) {
        //if (true) {
           std::cout << "Target found!" << std::endl;
        
           // Coords should be scaled to the original image. The coords from the model are relative to the model's input height and width.
           x1 = ((x1- pad_size_x)  / model_width_after_padding) * originalImageSize.width ;
           x2 = ((x2- pad_size_x) / model_width_after_padding) * originalImageSize.width ;
           y1 = ((y1 - pad_size_y) / model_height_after_padding) * originalImageSize.height ;
           y2 = ((y2 - pad_size_y) / model_height_after_padding) * originalImageSize.height ;
           cv::Rect clipped = clipBox(x1, y1, x2, y2, originalImageSize.width, originalImageSize.height);
	   std::cout << "Class Name: " << classNames.at(classPrediction) << std::endl;
           std::cout << "Coords: Top Left (" << clipped.x << ", " << clipped.x + clipped.width << "), Bottom Right (" << clipped.y << ", " << clipped.y + clipped.height << ")" << std::endl;
           std::cout << "Accuracy: " << accuracy << std::endl;

           Result result(clipped.x, clipped.x + clipped.width, clipped.y, clipped.y + clipped.height, classPrediction, accuracy);

           resultVector.push_back( result );

           std::cout << std::endl;
	}
    }

    return resultVector;
}


cv::Mat YoloDetect::preprocess(cv::Mat& image, int model_input_width, int model_input_height){

    // Channels order: BGR to RGB
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    // Calculate the scaling factor for resizing without distortion
    double scale;
    if (image.cols / static_cast<double>(image.rows) > model_input_width / static_cast<double>(model_input_height)) {
        scale = model_input_width / static_cast<double>(image.cols);
    } else {
        scale = model_input_height / static_cast<double>(image.rows);
    }

    // Resize the image with keeping the aspect ratio
    cv::Mat resizedImage;
    cv::resize(image, resizedImage, cv::Size(), scale, scale);

    model_height_after_padding = resizedImage.size[0];
    model_width_after_padding = resizedImage.size[1];
    // Create a blank canvas with the desired model input size
    cv::Mat paddedImage = cv::Mat::zeros(model_input_height, model_input_width, resizedImage.type());

    // Calculate the position to paste the resized image
    int x_offset = (paddedImage.cols - resizedImage.cols) / 2;
    int y_offset = (paddedImage.rows - resizedImage.rows) / 2;
    pad_size_y = y_offset;
    pad_size_x = x_offset;

    // Copy the resized image to the center of the canvas
    resizedImage.copyTo(paddedImage(cv::Rect(x_offset, y_offset, resizedImage.cols, resizedImage.rows)));
    // Convert image to float32 and normalize
    cv::Mat floatImage;
    paddedImage.convertTo(floatImage, CV_32F, 1.0 / 255.0);

    // Create a 4-dimensional blob from the image
    cv::Mat blobImage = cv::dnn::blobFromImage(floatImage);


    return blobImage;
}

cv::Mat YoloDetect::drawBoundingBox(cv::Mat& image, std::vector<Result>& resultVector){
    for( auto result : resultVector ) {

        if( result.accuracy > ACCURACY ) { // Threshold, can be made function parameter

            cv::rectangle(image, cv::Point(result.x1, result.y1), cv::Point(result.x2, result.y2), cv::Scalar(0, 255, 0), 2);

            cv::putText(image, classNames.at( result.obj_id ),
                        cv::Point(result.x1, result.y1 - 3), cv::FONT_ITALIC,
                        0.8, cv::Scalar(255, 255, 255), 2);

            cv::putText(image, std::to_string(result.accuracy),
                        cv::Point(result.x1, result.y1+30), cv::FONT_ITALIC,
                        0.8, cv::Scalar(255, 255, 0), 2);
        }
    }
    return image;

}

cv::Rect YoloDetect::clipBox(float x1, float y1, float x2, float y2, int imageWidth, int imageHeight) {
    int x1_clipped = std::max(0, std::min(static_cast<int>(x1), imageWidth - 1));
    int y1_clipped = std::max(0, std::min(static_cast<int>(y1), imageHeight - 1));
    int x2_clipped = std::max(0, std::min(static_cast<int>(x2), imageWidth - 1));
    int y2_clipped = std::max(0, std::min(static_cast<int>(y2), imageHeight - 1));
    return cv::Rect(cv::Point(x1_clipped, y1_clipped), cv::Point(x2_clipped, y2_clipped));
}

std::vector<Ort::Value> YoloDetect::RunInference(cv::Mat& inputImage){
    
    size_t num_input_nodes = session->GetInputCount();
    size_t num_output_nodes = session->GetOutputCount();
    std::vector<int64_t> inputDims = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
                OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(memoryInfo,
                                                              inputImage.ptr<float>(),
                                                              inputImage.total() * sizeof(float),
                                                              inputDims.data(),
                                                              inputDims.size());
    
    std::vector<Ort::Value> outputTensors = session->Run(Ort::RunOptions{nullptr},
                                                        &input_node_name,
                                                        &inputTensor,
                                                        num_input_nodes,
                                                        &output_node_name,
                                                        num_output_nodes);
    return outputTensors;
}

