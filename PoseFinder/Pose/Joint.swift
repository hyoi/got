/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Implementation details of a structure used to describe a joint.
*/

import CoreGraphics

class Joint {
    enum Name: Int, CaseIterable {
        case nose
        case leftEye
        case rightEye
        case leftEar
        case rightEar
        case leftShoulder
        case rightShoulder
        case leftElbow
        case rightElbow
        case leftWrist
        case rightWrist
        case leftHip
        case rightHip
        case leftKnee
        case rightKnee
        case leftAnkle
        case rightAnkle
    }

    ///使用可能なジョイントの総数。
    /// The total number of joints available.
    static var numberOfJoints: Int {
        return Name.allCases.count
    }

    /// The name used to identify the joint.
    let name: Name
    //画像に対する関節の位置。
    ///
    ///位置は最初にモデルの入力画像サイズを基準にしており、次に元の画像にマッピングされます
    ///関連するポーズを作成した後のサイズ。
    /// The position of the joint relative to the image.
    ///
    /// The position is initially relative to the model's input image size and then mapped to the original image
    /// size after constructing the associated pose.
    var position: CGPoint

    ///モデルの出力グリッドへのジョイントのそれぞれのセルインデックス。
    /// The joint's respective cell index into model's output grid.
    var cell: PoseNetOutput.Cell

    ///このジョイントに関連付けられた信頼スコア。
    ///
    ///共同信頼度は、PoseNetモデルによって出力された `heatmap`配列から取得されます。
    /// The confidence score associated with this joint.
    ///
    /// The joint confidence is obtained from the `heatmap` array output by the PoseNet model.
    var confidence: Double

    /// A boolean value that indicates if the joint satisfies the joint threshold defined in the configuration.
    ///ジョイントが構成で定義されたジョイントしきい値を満たすかどうかを示すブール値。
    var isValid: Bool

    init(name: Name,
         cell: PoseNetOutput.Cell = .zero,
         position: CGPoint = .zero,
         confidence: Double = 0,
         isValid: Bool = false) {
        self.name = name
        self.cell = cell
        self.position = position
        self.confidence = confidence
        self.isValid = isValid
    
    }
}
