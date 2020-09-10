/*
See LICENSE folder for this sample’s licensing information.

Abstract:
Implementation details of a structure used to describe a pose.
*/

import CoreGraphics

struct Pose {
    /// 2つのジョイント間の親子関係を記述するために使用される構造。
    /// A structure used to describe a parent-child relationship between two joints.
    struct Edge {
        let index: Int
        let parent: Joint.Name
        let child: Joint.Name

        init(from parent: Joint.Name, to child: Joint.Name, index: Int) {
            self.index = index
            self.parent = parent
            self.child = child
        }
    }

    ///ジョイント間の接続を定義するために使用されるエッジの配列。
        ///
        ///インデックスは、変位マップ内の関連する値にアクセスするために使用されるインデックスに関連しています
        /// PoseNetモデルによる出力
    /// An array of edges used to define the connections between the joints.
    ///
    /// The index relates to the index used to access the associated value within the displacement maps
    /// output by the PoseNet model.
    static let edges = [
        Edge(from: .nose, to: .leftEye, index: 0),
        Edge(from: .leftEye, to: .leftEar, index: 1),
        Edge(from: .nose, to: .rightEye, index: 2),
        Edge(from: .rightEye, to: .rightEar, index: 3),
        Edge(from: .nose, to: .leftShoulder, index: 4),
        Edge(from: .leftShoulder, to: .leftElbow, index: 5),
        Edge(from: .leftElbow, to: .leftWrist, index: 6),
        Edge(from: .leftShoulder, to: .leftHip, index: 7),
        Edge(from: .leftHip, to: .leftKnee, index: 8),
        Edge(from: .leftKnee, to: .leftAnkle, index: 9),
        Edge(from: .nose, to: .rightShoulder, index: 10),
        Edge(from: .rightShoulder, to: .rightElbow, index: 11),
        Edge(from: .rightElbow, to: .rightWrist, index: 12),
        Edge(from: .rightShoulder, to: .rightHip, index: 13),
        Edge(from: .rightHip, to: .rightKnee, index: 14),
        Edge(from: .rightKnee, to: .rightAnkle, index: 15)
    ]
    ///ポーズを構成する関節。
    /// The joints that make up a pose.
    private(set) var joints: [Joint.Name: Joint] = [
        .nose: Joint(name: .nose),
        .leftEye: Joint(name: .leftEye),
        .leftEar: Joint(name: .leftEar),
        .leftShoulder: Joint(name: .leftShoulder),
        .leftElbow: Joint(name: .leftElbow),
        .leftWrist: Joint(name: .leftWrist),
        .leftHip: Joint(name: .leftHip),
        .leftKnee: Joint(name: .leftKnee),
        .leftAnkle: Joint(name: .leftAnkle),
        .rightEye: Joint(name: .rightEye),
        .rightEar: Joint(name: .rightEar),
        .rightShoulder: Joint(name: .rightShoulder),
        .rightElbow: Joint(name: .rightElbow),
        .rightWrist: Joint(name: .rightWrist),
        .rightHip: Joint(name: .rightHip),
        .rightKnee: Joint(name: .rightKnee),
        .rightAnkle: Joint(name: .rightAnkle)
    ]

    ///このポーズに関連付けられた信頼スコア。
    /// The confidence score associated with this pose.
    var confidence: Double = 0.0

    /// Accesses the joint with the specified name.
    subscript(jointName: Joint.Name) -> Joint {
        get {
            assert(joints[jointName] != nil)
            return joints[jointName]!
        }
        set {
            joints[jointName] = newValue
        }
    }

    /// ** from **または** to **指定されたジョイントをリンクするすべてのエッジを返します。
        ///
        /// - パラメーター：
        ///-jointName：ジョイント名を照会します。
        ///-戻り値： `jointName`に接続または接続するすべてのエッジ。
    /// Returns all edges that link **from** or **to** the specified joint.
    ///
    /// - parameters:
    /// - jointName: Query joint name.
    /// - returns: All edges that connect to or from `jointName`.
    static func edges(for jointName: Joint.Name) -> [Edge] {
        return Pose.edges.filter {
            $0.parent == jointName || $0.child == jointName
        }
    }
    ///指定した親と子のジョイント名を持つエッジを返します。
    ///
    /// - パラメーター：
    ///-parentJointName：エッジの親ジョイント名。
    ///-childJointName：エッジの子ジョイント名。
    ///-戻り値： `jointName`に接続または接続するすべてのエッジ。
    /// Returns the edge having the specified parent and child  joint names.
    ///
    /// - parameters:
    ///     - parentJointName: Edge's parent joint name.
    ///     - childJointName: Edge's child joint name.
    /// - returns: All edges that connect to or from `jointName`.
    static func edge(from parentJointName: Joint.Name, to childJointName: Joint.Name) -> Edge? {
        return Pose.edges.first(where: { $0.parent == parentJointName && $0.child == childJointName })
    }
    
}
