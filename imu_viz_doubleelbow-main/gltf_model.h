#pragma once

#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

class GltfModel {
public:
    bool load(const std::string& path);
    bool isLoaded() const { return loaded; }

    void draw(const glm::quat& leftForearmQ,
              const glm::quat& rightForearmQ,
              const glm::quat& leftUpperArmQ,
              const glm::quat& rightUpperArmQ,
              const glm::quat& leftThighQ,
              const glm::quat& rightThighQ,
              const glm::quat& leftShinQ,
              const glm::quat& rightShinQ,
              const glm::quat& hipsQ,
              const glm::quat& chestQ);

private:
    struct Vertex {
        glm::vec3 position = glm::vec3(0.0f);
        glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec4 weights = glm::vec4(0.0f);
        glm::ivec4 joints = glm::ivec4(0);
    };

    struct Primitive {
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        glm::vec4 color = glm::vec4(0.82f, 0.72f, 0.62f, 1.0f);
    };

    struct MeshInstance {
        int nodeIndex = -1;
        int skinIndex = -1;
        std::vector<Primitive> primitives;
    };

    struct Node {
        std::string name;
        int parent = -1;
        std::vector<int> children;
        glm::vec3 translation = glm::vec3(0.0f);
        glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 scale = glm::vec3(1.0f);
        glm::mat4 localMatrix = glm::mat4(1.0f);
        bool hasMatrix = false;
    };

    struct Skin {
        std::vector<int> joints;
        std::vector<glm::mat4> inverseBindMatrices;
    };

    struct BoneTarget {
        int node = -1;
        int child = -1;
        glm::quat alignment = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        bool valid = false;
    };

    bool loaded = false;
    std::vector<Node> nodes;
    std::vector<Skin> skins;
    std::vector<MeshInstance> meshes;
    std::vector<int> rootNodes;
    std::vector<glm::mat4> bindGlobals;
    std::vector<glm::mat4> animatedGlobals;
    std::vector<glm::quat> animatedLocalRotations;

    BoneTarget leftArm;
    BoneTarget leftForeArm;
    BoneTarget rightArm;
    BoneTarget rightForeArm;
    BoneTarget leftThigh;
    BoneTarget leftShin;
    BoneTarget rightThigh;
    BoneTarget rightShin;
    BoneTarget hips;
    BoneTarget chest;
    
    void updateTorsoBones(const glm::quat& hipsQ, const glm::quat& chestQ);

    glm::mat4 composeLocal(const Node& node, const glm::quat& localRotation) const;
    void computeGlobals();
    void computeGlobalsRecursive(int nodeIndex, const glm::mat4& parentMatrix);
    void updateArmBones(const glm::quat& leftForearmQ,
                        const glm::quat& rightForearmQ,
                        const glm::quat& leftUpperArmQ,
                        const glm::quat& rightUpperArmQ);
    void updateLegBones(const glm::quat& leftThighQ,
                        const glm::quat& rightThighQ,
                        const glm::quat& leftShinQ,
                        const glm::quat& rightShinQ);
    void applyWorldRotation(const BoneTarget& target, const glm::quat& worldRotation);
    void applyWorldDirection(const BoneTarget& target, const glm::vec3& worldDirection);
    void prepareBoneTarget(BoneTarget& target, const char* nodeName, const char* childName);
    int findNode(const std::string& name) const;
    glm::vec3 nodeWorldPosition(int nodeIndex) const;
    glm::quat nodeWorldRotation(int nodeIndex) const;
    glm::quat rotationBetween(glm::vec3 from, glm::vec3 to) const;
    glm::mat4 skinMatrix(const Skin& skin, int jointSlot) const;
    void drawPrimitive(const Primitive& primitive, const Skin* skin);
};
