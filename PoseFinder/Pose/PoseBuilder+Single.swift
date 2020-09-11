/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The implementation of a single-person pose estimation algorithm, based on the TensorFlow
 project "Pose Detection in the Browser."
*/

import CoreGraphics

extension PoseBuilder {

    /// PoseNetモデルからの出力を使用して構築されたポーズを返します。
    /// Returns a pose constructed using the outputs from the PoseNet model.
    var pose: Pose {
        var pose = Pose()
        //各関節について、最も可能性の高い位置と関連する信頼度を見つけます
        //最大のセルのヒートマップ配列をクエリする
        //信頼度とこれを使用して位置を計算します。
        // For each joint, find its most likely position and associated confidence
        // by querying the heatmap array for the cell with the greatest
        // confidence and using this to compute its position.
        pose.joints.values.forEach { joint in
            configure(joint: joint)
        }
        
        //ポーズの信頼度を計算して割り当てます。
        // Compute and assign the confidence for the pose.
        pose.confidence = pose.joints.values
            .map { $0.confidence }.reduce(0, +) / Double(Joint.numberOfJoints)

        // Map the pose joints positions back onto the original image.
        pose.joints.values.forEach { joint in
            joint.position = joint.position.applying(modelToInputTransformation)
        }
      
        return pose
    }

    ///最も信頼できる関連セルを使用してジョイントのプロパティを設定します。
    ///
    ///信頼度は、PoseNetモデルによって出力された `heatmap`配列から取得されます。
    /// - パラメーター：
    ///-ジョイント：更新するジョイント。
    private func configure(joint: Joint) {
          print("ポーズの中身",pose)
        //ヒートマップの関連するジョイントチャネルを反復処理して、
        //最も信頼できるセル。
        var bestCell = PoseNetOutput.Cell(0, 0)
        var bestConfidence = 0.0
        for yIndex in 0..<output.height {
            for xIndex in 0..<output.width {
                let currentCell = PoseNetOutput.Cell(yIndex, xIndex)
                let currentConfidence = output.confidence(for: joint.name, at: currentCell)

                // Keep track of the cell with the greatest confidence.
                if currentConfidence > bestConfidence {
                    bestConfidence = currentConfidence
                    bestCell = currentCell
                }
            }
        }

        // Update joint.
        joint.cell = bestCell
        joint.position = output.position(for: joint.name, at: joint.cell)
        joint.confidence = bestConfidence
        joint.isValid = joint.confidence >= configuration.jointConfidenceThreshold
   
    }
    
   
}
