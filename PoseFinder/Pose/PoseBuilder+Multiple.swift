/*
See LICENSE folder for this sample’s licensing information.

Abstract:
The implementation of a multiple-person pose estimation algorithm, based on the TensorFlow
 project "Pose Detection in the Browser."
*/

import CoreGraphics

extension PoseBuilder {

    /// PoseNetモデルからの出力を使用して構築されたポーズの配列を返します。
    /// Returns an array of poses constructed using the outputs from the PoseNet model.
    var poses: [Pose] {
        var detectedPoses = [Pose]()
        //最大の信頼度で関節を反復します。ここでは、
        //候補ルート。それぞれを開始点として使用して、ポーズを組み立てます。

        // Iterate through the joints with the greatest confidence, referred to here as
        // candidate roots, using each as a starting point to assemble a pose.
        for candidateRoot in candidateRoots {
            //の関節の近くにある候補を無視します
            //同じタイプで、既存のポーズにすでに割り当てられています
            // Ignore any candidates that are in the proximity of joints of the
            // same type and have already been assigned to an existing pose.
            let maxDistance = configuration.matchingJointDistance
            guard !detectedPoses.contains(candidateRoot, within: maxDistance) else {
                continue
            }

            var pose = assemblePose(from: candidateRoot)
            //すべての合計を割ってポーズの信頼度を計算します
            //既存のポーズの重複しないジョイントを合計で
            //ジョイントの数。
            // Compute the pose's confidence by dividing the sum of all
            // non-overlapping joints, from existing poses, by the total
            // number of joints.
            pose.confidence = confidence(for: pose, detectedPoses: detectedPoses)

            // Ignore any pose that has a confidence less than the assigned threshold.
            guard pose.confidence >= configuration.poseConfidenceThreshold else {
                continue
            }

            detectedPoses.append(pose)

            // Exit early if enough poses have been detected.
            if detectedPoses.count >= configuration.maxPoseCount {
                break
            }
        }

        //ポーズジョイントの位置を元の画像にマッピングします
        //事前に計算された変換行列。
        // Map the pose joints positions back onto the original image using
        // the pre-computed transformation matrix.
        detectedPoses.forEach { pose in
            pose.joints.values.forEach { joint in
                joint.position = joint.position.applying(modelToInputTransformation)
            }
        }
print("ディテクティッドポージズの数",detectedPoses)
        return detectedPoses
    }

    
    
    
    ///ポーズを組み立てるためのルートとして使用される候補ジョイントを返します。
    ///
    ///このプロパティは、 `heatmap`配列を検索して、最も信頼できるジョイントを見つけます。
    ///候補の配列を降順で返します。
    ///-戻り値：信頼度に基づいて降順の候補関節の配列。

    /// Returns candidate joints that are used as roots to assemble poses.
    ///
    /// This property searches the `heatmap` array to find the joints with the greatest confidence,
    /// returning the array of candidates in descending order.
    /// - returns: An ordered array of candidate joints in descending order based on its confidence.
    private var candidateRoots: [Joint] {
        var candidateRoots = [Joint]()

        for jointName in Joint.Name.allCases {
            for yIndex in 0..<output.height {
                for xIndex in 0..<output.width {
                    let cell = PoseNetOutput.Cell(yIndex, xIndex)

                    let jointConfidence = output.confidence(for: jointName, at: cell)

                    guard jointConfidence >= configuration.jointConfidenceThreshold
                        else { continue }

                    // Only consider a joint whose score is the greatest among its neighbors.
                    let greatestNeighborsConfidence = greatestConfidence(for: jointName, at: cell)
                    guard jointConfidence >= greatestNeighborsConfidence
                        else { continue }

                    let candidate = Joint(name: jointName,
                                          cell: cell,
                                          position: output.position(for: jointName, at: cell),
                                          confidence: jointConfidence,
                                          isValid: true)

                    candidateRoots.append(candidate)
                }
            }
        }

        //信頼できる順に候補を並べ替えて返します。
        // Sort and return candidates in order of their confidence.
        return candidateRoots.sorted { $0.confidence > $1.confidence }
    }
    ///与えられたポーズの信頼度を計算して返します。
    ///
    ///オーバーラップしていないすべてのジョイントの合計を以下から除算することにより、ポーズの信頼度を計算します
    ///ジョイントの総数による既存のポーズ。
    /// - パラメーター：
    ///-ポーズ：信頼度を計算するポーズ。
    ///-detectedPoses：すでに検出されたポーズの配列。
    ///-戻り値：指定されたポーズの信頼度。
    /// Calculates and returns the given pose's confidence.
    ///
    /// Calculates the pose's confidence by dividing the sum of all non-overlapping joints, from
    /// existing poses, by the total number of joints.
    /// - parameters:
    ///     - pose: Pose to compute the confidence for.
    ///     - detectedPoses: An array of poses already detected.
    /// - returns: The given pose's confidence.
    private func confidence(for pose: Pose, detectedPoses: [Pose]) -> Double {

        // Find all non-overlapping joints belonging to the existing pose.
        let joints = nonOverlappingJoints(for: pose, detectedPoses: detectedPoses)
        print("joints.mapは、",Double())
        
        return joints.map { $0.confidence }.reduce(0, +) / Double(Joint.numberOfJoints)
    }

    ///指定されたポーズのすべてのオーバーラップしていないジョイントを返します。
    ///
    ///重複しないジョイントは、 `configuration.matchingJointDistance`よりも距離が長いジョイントです
    ///既存のポーズに属する同じタイプのジョイントから。
    /// - パラメーター：
    ///-ポーズ：オーバーラップしていないジョイントを見つけるためのポーズ。
    ///-detectedPoses：既存のポーズの配列。
    ///-戻り値： `pose`の重複しないジョイント。
    /// Returns all non-overlapping joints for a given pose.
    ///
    /// Non-overlapping joints are joints that have a distance greater than `configuration.matchingJointDistance`
    /// from any joint of the same type belonging to an existing pose.
    /// - parameters:
    ///     - pose: Pose to find non-overlapping joints for.
    ///     - detectedPoses: An array of existing poses.
    /// - returns: Non-overlapping joints for `pose`.
    private func nonOverlappingJoints(for pose: Pose, detectedPoses: [Pose]) -> [Joint] {
        return pose.joints.values.filter { joint in
            guard joint.isValid else {
                return false
            }

            for detectedPose in detectedPoses {
                let otherJoint = detectedPose[joint.name]

                guard otherJoint.isValid else {
                    continue
                }

                if joint.position.distance(to: otherJoint.position) <= configuration.matchingJointDistance {
                    return false
                }
            }
            return true
        }
    }

    ///与えられたセルの周りの最大の信頼度を持つ関節の信頼度を返します。
        ///
        /// - パラメーター：
        ///-jointName：照会されるジョイントの名前。
        ///-セル：検索する特定のジョイントの座標。
        ///-戻り値：最大の信頼値。
    /// Returns the confidence of the joint with the greatest confidence around the given cell.
    ///
    /// - parameters:
    ///     - jointName: Name of joint being queried.
    ///     - cell: The coordinates of the given joint to search around.
    /// - returns: The greatest confidence value.
    private func greatestConfidence(for jointName: Joint.Name, at cell: PoseNetOutput.Cell) -> Double {
        //ローカルウィンドウの開始インデックスと終了インデックスを計算します
        // Calculate the start and end indices for the local window.
        let yLowerBound = max(cell.yIndex - configuration.localSearchRadius, 0)
        let yUpperBound = min(cell.yIndex + configuration.localSearchRadius, output.height - 1)
        let yWindowIndices: ClosedRange<Int> = yLowerBound...yUpperBound

        let xLowerBound = max(cell.xIndex - configuration.localSearchRadius, 0)
        let xUpperBound = min(cell.xIndex + configuration.localSearchRadius, output.width - 1)
        let xWindowIndices: ClosedRange<Int> = xLowerBound...xUpperBound

        var greatestConfidence = 0.0

        //信頼性の高いセルを探すためにローカルウィンドウをスキャンします。        // Scan over the local window in search of the cell with the greatest confidence.
        for yIndex in yWindowIndices {
            for xIndex in xWindowIndices {
                guard yIndex != cell.yIndex, xIndex != cell.xIndex else {
                    continue
                }
                let localCell = PoseNetOutput.Cell(yIndex, xIndex)
                let localConfidence = output.confidence(for: jointName, at: localCell)

                greatestConfidence = max(greatestConfidence, localConfidence)
            }
        }
print("最大の信頼値は、",greatestConfidence)
        return greatestConfidence
    }

    ///与えられた候補ルートジョイントを使用してポーズを組み立てます。
        ///
        ///ポーズは、PoseNetモデルによって出力されたディスプレイスメントマップをトラバースして隣接するジョイントを見つけることで組み立てられます。
        ///最初は候補ルートジョイント。その後、その後のすべての関節が発見されます。
        ///
        /// - パラメーター：
        ///-rootJoint：ルートジョイントへの参照。
        ///-戻り値：与えられた候補ルートジョイントを使用して組み立てられたポーズ。
    /// Assembles a pose using the given candidate root joint.
    ///
    /// The pose is assembled by traversing the displacement maps output by the PoseNet model to find adjacent joints,
    /// initially for the candidate root joint. Then all subsequent joints are discovered after that.
    ///
    /// - parameters:
    ///     - rootJoint: Reference to the root joint.
    /// - returns: Pose assembled using the given candidate root joint.
    private func assemblePose(from rootJoint: Joint) -> Pose {
        // Create a pose and update its root joint.
        var pose = Pose()
        pose[rootJoint.name] = rootJoint

        // Update the remaining joints by spawning from the root joint to find
        // adjacent joints using the displacement maps output by the PoseNet model.
        var queryJoints = [rootJoint]
        while !queryJoints.isEmpty {
            let joint = queryJoints.removeFirst()

            // Update the details of all the adjacent joints.
            for edge in Pose.edges(for: joint.name) {
                let parentJoint = pose[edge.parent]
                let childJoint = pose[edge.child]

                // Ignore any edges that have already been processed.
                guard !(parentJoint.isValid && childJoint.isValid) else {
                    continue
                }

                // Set the source joint to search from.
                let sourceJoint = parentJoint.isValid ? parentJoint : childJoint
                // Set the adjacent joint to search to.
                let adjacentJoint = parentJoint.isValid ? childJoint : parentJoint
                // Update the properties of the adjacent joint.
                configure(joint: adjacentJoint,
                          from: sourceJoint,
                          given: Pose.edge(from: parentJoint.name, to: childJoint.name)!)
                // Add the adjacent joint to the queue if its confidence is greater than
                // the joint confidence threshold.
                if adjacentJoint.isValid {
                    queryJoints.append(adjacentJoint)
                }
            }
        }
print("組み立てられたポーズ。",pose)
        return pose
    }

    
    

    /// `sourceJoint`および関連する` edge`を使用して、指定されたジョイントのプロパティを更新します。
        ///
        ///ジョイントのプロパティ（ `cell`、` position`、および `confidence`）は、最初にジョイントの推論によって取得されます
        /// `sourceJoint`の位置と、
        ///変位マップ。この位置は、セルインデックスに変換され、関節の信頼度を取得するために使用されます。
        ///
        /// - パラメーター：
        ///-joint：プロパティが更新されるジョイント。
        ///-sourceJoint：検索元の有効なソースジョイント。
        ///-エッジ： `joint`と` sourceJoint`の間の関連エッジ。
    /// Update the properties of the given joint using the `sourceJoint` and associated `edge`.
    ///
    /// The properties (`cell`, `position`, and `confidence`) of the joint are obtained by first inferring the joint’s
    /// likely position using the position of the `sourceJoint` and displacement vector encoded in the
    /// displacement maps. This position is then converted into a cell index and used to obtain the joint's confidence.
    ///
    /// - parameters:
    ///     - joint: Joint whose properties are updated.
    ///     - sourceJoint: Valid source joint to search from.
    ///     - edge: Associated edge between the `joint` and `sourceJoint`.
    private func configure(joint: Joint,
                           from sourceJoint: Joint,
                           given edge: Pose.Edge) {

        // Query the appropriate displacement map to obtain the displacement vector.
        var displacementVector = CGVector.zero
        if edge.parent == sourceJoint.name {
            // Parent -> Child.
            displacementVector = output.forwardDisplacement(for: edge.index, at: sourceJoint.cell)
        } else {
            // Child -> Parent.
            displacementVector = output.backwardDisplacement(for: edge.index, at: sourceJoint.cell)
        }

        // Apply the displacement vector to the source joint's position to find an
        // approximate position of the joint.
        var approximateJointPosition = sourceJoint.position + displacementVector

        // Refine the joint's position by adjusting the position using the associated
        // offset obtained from the offsets array.
        for _ in 0..<configuration.adjacentJointOffsetRefinementSteps {
            guard let jointCell = output.cell(for: approximateJointPosition) else {
                break
            }

            let offset = output.offset(for: joint.name, at: jointCell)
            approximateJointPosition.x = CGFloat(jointCell.xIndex) * CGFloat(output.modelOutputStride) + offset.dx
            approximateJointPosition.y = CGFloat(jointCell.yIndex) * CGFloat(output.modelOutputStride) + offset.dy
        }

        // Obtain and assign the joint's cell by mapping the derived position
        // back to an index.
        guard let jointCell = output.cell(for: approximateJointPosition) else {
            return
        }

        // Update joint.
        joint.cell = jointCell
        joint.position = approximateJointPosition
        joint.confidence = output.confidence(for: joint.name, at: joint.cell)
        joint.isValid = joint.confidence >= configuration.jointConfidenceThreshold
    }
}

// MARK: - Array Extension

private extension Array where Element == Pose {
    
    
    ///指定された候補ジョイントが既存のジョイントと一致するかどうかを示すブール値を返します
    ///配列のポーズ。
    ///
    ///このメソッドは、配列内の各ポーズを反復処理して、候補と同じタイプの既知の関節をチェックし、
    ///そして、それらの間の距離が `distance`で定義された特定の量よりも小さい場合、` true`を返します。
    ///
    /// - パラメーター：
    ///-候補：候補のジョイント。
    ///-距離：2つのジョイントがほぼ同じかどうかを決定するために使用されるしきい値。
    ///-戻り値：候補がいずれかの「ポーズ」の同等の関節と一致する場合は「true」、それ以外の場合は「false」。
    /// Returns a Boolean value that indicates whether the given candidate joint matches an existing joint of
    /// a pose in the array.
    ///
    /// This method iterates through each pose in the array checking its known joints of the same type as the candidate,
    /// and returns `true` if the distance between them is less than a specific amount, defined by `distance`.
    ///
    /// - parameters:
    ///     - candidate: A candidate joint.
    ///     - distance: The threshold used to determine if two joints are approximately the same.
    /// - returns: `true` if the candidate matches the equivalent joint in any `Pose`, otherwise `false`.
    func contains(_ candidate: Joint, within distance: Double) -> Bool {
        // Check each pose in the array.
        for pose in self {
            // Find the joint that matches the candidate.
            let matchingJoint = pose[candidate.name]

            // Skip the pose's joint if it is not valid.
            guard matchingJoint.isValid else { continue }

            if matchingJoint.position.distance(to: candidate.position) <= distance {
                // The candidate is in proximity to an existing joint.
                return true
            }
        }

        // None of the poses had a matching joint in the location of the candidate.
        return false
    }
}
