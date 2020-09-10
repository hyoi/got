/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The implementation of a structure that analyzes the PoseNet model outputs to detect
 single or multiple poses.
*/

import CoreGraphics

struct PoseBuilder {
    /// PoseNetモデルからの予測。
    ///
    ///予測出力は、ポーズを見つけて構築するために分析されます。
    /// A prediction from the PoseNet model.
    ///
    /// Prediction outputs are analyzed to find and construct poses.
    let output: PoseNetOutput
    ///ジョイントをPoseNetモデルの入力画像サイズから元の画像サイズにマッピングするために使用される変換行列。
    /// A transformation matrix used to map joints from the PoseNet model's input image size onto the original image size.
    let modelToInputTransformation: CGAffineTransform
    ///ポーズビルダーがポーズアルゴリズムで使用するパラメーター。
    /// The parameters the Pose Builder uses in its pose algorithms.
    var configuration: PoseBuilderConfiguration

    init(output: PoseNetOutput, configuration: PoseBuilderConfiguration, inputImage: CGImage) {
        self.output = output
        self.configuration = configuration
        
        //変換行列を作成して、関節の位置を空間に戻す
        //元の入力サイズの。
        // Create a transformation matrix to transform joint positions back into the space
        // of the original input size.
        modelToInputTransformation = CGAffineTransform(scaleX: inputImage.size.width / output.modelInputSize.width,
                                                       y: inputImage.size.height / output.modelInputSize.height)
    }
}
