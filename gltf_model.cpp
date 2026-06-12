#define CGLTF_IMPLEMENTATION
#include "third_party/cgltf.h"

#include "gltf_model.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace {
glm::mat4 readMat4(const cgltf_accessor* accessor, cgltf_size index)
{
    cgltf_float values[16] = {};
    cgltf_accessor_read_float(accessor, index, values, 16);
    glm::mat4 result(1.0f);
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            result[c][r] = values[c * 4 + r];
        }
    }
    return result;
}

glm::mat4 nodeLocalFromCgltf(const cgltf_node& node)
{
    if (node.has_matrix) {
        glm::mat4 matrix(1.0f);
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                matrix[c][r] = node.matrix[c * 4 + r];
            }
        }
        return matrix;
    }

    glm::vec3 t(0.0f);
    glm::quat r(1.0f, 0.0f, 0.0f, 0.0f);
    glm::vec3 s(1.0f);

    if (node.has_translation) {
        t = glm::vec3(node.translation[0], node.translation[1], node.translation[2]);
    }
    if (node.has_rotation) {
        r = glm::normalize(glm::quat(node.rotation[3], node.rotation[0], node.rotation[1], node.rotation[2]));
    }
    if (node.has_scale) {
        s = glm::vec3(node.scale[0], node.scale[1], node.scale[2]);
    }

    return glm::translate(glm::mat4(1.0f), t) * glm::toMat4(r) * glm::scale(glm::mat4(1.0f), s);
}

glm::vec4 materialColor(const cgltf_material* material)
{
    if (!material) {
        return glm::vec4(0.82f, 0.72f, 0.62f, 1.0f);
    }
    if (material->has_pbr_metallic_roughness) {
        const cgltf_float* c = material->pbr_metallic_roughness.base_color_factor;
        return glm::vec4(c[0], c[1], c[2], c[3]);
    }
    return glm::vec4(0.82f, 0.72f, 0.62f, 1.0f);
}
}

bool GltfModel::load(const std::string& path)
{
    cgltf_options options = {};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, path.c_str(), &data);
    if (result != cgltf_result_success) {
        std::cerr << "Failed to parse GLB: " << path << "\n";
        return false;
    }

    result = cgltf_load_buffers(&options, data, path.c_str());
    if (result != cgltf_result_success) {
        std::cerr << "Failed to load GLB buffers: " << path << "\n";
        cgltf_free(data);
        return false;
    }

    nodes.clear();
    skins.clear();
    meshes.clear();
    rootNodes.clear();

    std::unordered_map<const cgltf_node*, int> nodeMap;
    nodes.resize(data->nodes_count);
    for (cgltf_size i = 0; i < data->nodes_count; ++i) {
        const cgltf_node& src = data->nodes[i];
        Node& dst = nodes[i];
        dst.name = src.name ? src.name : "";
        dst.localMatrix = nodeLocalFromCgltf(src);
        dst.hasMatrix = src.has_matrix;
        if (src.has_translation) {
            dst.translation = glm::vec3(src.translation[0], src.translation[1], src.translation[2]);
        }
        if (src.has_rotation) {
            dst.rotation = glm::normalize(glm::quat(src.rotation[3], src.rotation[0], src.rotation[1], src.rotation[2]));
        }
        if (src.has_scale) {
            dst.scale = glm::vec3(src.scale[0], src.scale[1], src.scale[2]);
        }

        if (dst.name == "mixamorig:LeftShoulder" || dst.name == "mixamorig:RightShoulder") {
            constexpr float kShoulderWidthScale = 4.0f;
            dst.translation.x *= kShoulderWidthScale;
        }
        nodeMap[&src] = static_cast<int>(i);
    }

    for (cgltf_size i = 0; i < data->nodes_count; ++i) {
        const cgltf_node& src = data->nodes[i];
        Node& dst = nodes[i];
        if (src.parent) {
            dst.parent = nodeMap[src.parent];
        }
        for (cgltf_size c = 0; c < src.children_count; ++c) {
            dst.children.push_back(nodeMap[src.children[c]]);
        }
        if (dst.parent < 0) {
            rootNodes.push_back(static_cast<int>(i));
        }
    }

    skins.resize(data->skins_count);
    for (cgltf_size i = 0; i < data->skins_count; ++i) {
        const cgltf_skin& src = data->skins[i];
        Skin& dst = skins[i];
        dst.joints.resize(src.joints_count);
        dst.inverseBindMatrices.resize(src.joints_count, glm::mat4(1.0f));
        for (cgltf_size j = 0; j < src.joints_count; ++j) {
            dst.joints[j] = nodeMap[src.joints[j]];
            if (src.inverse_bind_matrices) {
                dst.inverseBindMatrices[j] = readMat4(src.inverse_bind_matrices, j);
            }
        }
    }

    for (cgltf_size n = 0; n < data->nodes_count; ++n) {
        const cgltf_node& node = data->nodes[n];
        if (!node.mesh) {
            continue;
        }

        MeshInstance instance;
        instance.nodeIndex = static_cast<int>(n);
        instance.skinIndex = node.skin ? static_cast<int>(node.skin - data->skins) : -1;

        for (cgltf_size p = 0; p < node.mesh->primitives_count; ++p) {
            const cgltf_primitive& srcPrim = node.mesh->primitives[p];
            if (srcPrim.type != cgltf_primitive_type_triangles) {
                continue;
            }

            const cgltf_accessor* pos = nullptr;
            const cgltf_accessor* norm = nullptr;
            const cgltf_accessor* weights = nullptr;
            const cgltf_accessor* joints = nullptr;
            for (cgltf_size a = 0; a < srcPrim.attributes_count; ++a) {
                const cgltf_attribute& attr = srcPrim.attributes[a];
                if (attr.type == cgltf_attribute_type_position) pos = attr.data;
                if (attr.type == cgltf_attribute_type_normal) norm = attr.data;
                if (attr.type == cgltf_attribute_type_weights) weights = attr.data;
                if (attr.type == cgltf_attribute_type_joints) joints = attr.data;
            }
            if (!pos) {
                continue;
            }

            Primitive prim;
            prim.color = materialColor(srcPrim.material);
            prim.vertices.resize(pos->count);
            for (cgltf_size v = 0; v < pos->count; ++v) {
                cgltf_float pv[3] = {};
                cgltf_accessor_read_float(pos, v, pv, 3);
                prim.vertices[v].position = glm::vec3(pv[0], pv[1], pv[2]);

                if (norm) {
                    cgltf_float nv[3] = {};
                    cgltf_accessor_read_float(norm, v, nv, 3);
                    prim.vertices[v].normal = glm::normalize(glm::vec3(nv[0], nv[1], nv[2]));
                }

                if (weights) {
                    cgltf_float wv[4] = {};
                    cgltf_accessor_read_float(weights, v, wv, 4);
                    prim.vertices[v].weights = glm::vec4(wv[0], wv[1], wv[2], wv[3]);
                }

                if (joints) {
                    cgltf_uint jv[4] = {};
                    cgltf_accessor_read_uint(joints, v, jv, 4);
                    for (int k = 0; k < 4; ++k) {
                        prim.vertices[v].joints[k] = static_cast<int>(jv[k]);
                    }
                }
            }

            if (srcPrim.indices) {
                prim.indices.resize(srcPrim.indices->count);
                for (cgltf_size i = 0; i < srcPrim.indices->count; ++i) {
                    prim.indices[i] = static_cast<unsigned int>(cgltf_accessor_read_index(srcPrim.indices, i));
                }
            } else {
                prim.indices.resize(prim.vertices.size());
                for (cgltf_size i = 0; i < prim.vertices.size(); ++i) {
                    prim.indices[i] = static_cast<unsigned int>(i);
                }
            }

            instance.primitives.push_back(std::move(prim));
        }

        if (!instance.primitives.empty()) {
            meshes.push_back(std::move(instance));
        }
    }

    animatedLocalRotations.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        animatedLocalRotations[i] = nodes[i].rotation;
    }

    bindGlobals.resize(nodes.size(), glm::mat4(1.0f));
    animatedGlobals.resize(nodes.size(), glm::mat4(1.0f));
    computeGlobals();
    bindGlobals = animatedGlobals;

    prepareBoneTarget(leftArm, "mixamorig:LeftArm", "mixamorig:LeftForeArm");
    prepareBoneTarget(leftForeArm, "mixamorig:LeftForeArm", "mixamorig:LeftHand");
    prepareBoneTarget(rightArm, "mixamorig:RightArm", "mixamorig:RightForeArm");
    prepareBoneTarget(rightForeArm, "mixamorig:RightForeArm", "mixamorig:RightHand");
    prepareBoneTarget(leftThigh,  "mixamorig:LeftUpLeg",   "mixamorig:LeftLeg");
    prepareBoneTarget(leftShin,   "mixamorig:LeftLeg",     "mixamorig:LeftFoot");
    prepareBoneTarget(rightThigh, "mixamorig:RightUpLeg",  "mixamorig:RightLeg");
    prepareBoneTarget(rightShin,  "mixamorig:RightLeg",    "mixamorig:RightFoot");
    prepareBoneTarget(hips,  "mixamorig:Hips",   "mixamorig:Spine");
    prepareBoneTarget(chest, "mixamorig:Spine2",  "mixamorig:Neck");

    loaded = !meshes.empty();
    cgltf_free(data);

    if (loaded) {
        std::cout << "Loaded animated GLB: " << path << "\n";
    }
    return loaded;
}

glm::mat4 GltfModel::composeLocal(const Node& node, const glm::quat& localRotation) const
{
    if (node.hasMatrix) {
        return node.localMatrix;
    }
    return glm::translate(glm::mat4(1.0f), node.translation)
         * glm::toMat4(glm::normalize(localRotation))
         * glm::scale(glm::mat4(1.0f), node.scale);
}

void GltfModel::computeGlobals()
{
    for (int root : rootNodes) {
        computeGlobalsRecursive(root, glm::mat4(1.0f));
    }
}

void GltfModel::computeGlobalsRecursive(int nodeIndex, const glm::mat4& parentMatrix)
{
    animatedGlobals[nodeIndex] = parentMatrix * composeLocal(nodes[nodeIndex], animatedLocalRotations[nodeIndex]);
    for (int child : nodes[nodeIndex].children) {
        computeGlobalsRecursive(child, animatedGlobals[nodeIndex]);
    }
}

void GltfModel::draw(const glm::quat& leftForearmQ,
                     const glm::quat& rightForearmQ,
                     const glm::quat& leftUpperArmQ,
                     const glm::quat& rightUpperArmQ,
                     const glm::quat& leftThighQ,
                     const glm::quat& rightThighQ,
                     const glm::quat& leftShinQ,
                     const glm::quat& rightShinQ,
                     const glm::quat& hipsQ,
                     const glm::quat& chestQ)
{
    if (!loaded) {
        return;
    }

    updateArmBones(leftForearmQ, rightForearmQ, leftUpperArmQ, rightUpperArmQ);
    updateTorsoBones(hipsQ, chestQ);
    updateLegBones(leftThighQ, rightThighQ, leftShinQ, rightShinQ);

    glPushMatrix();
    glScalef(9.7f, 9.7f, 9.7f);
    glTranslatef(0.0f, 0.0f, 0.0f);

    for (const MeshInstance& mesh : meshes) {
        const Skin* skin = (mesh.skinIndex >= 0 && mesh.skinIndex < static_cast<int>(skins.size()))
            ? &skins[mesh.skinIndex]
            : nullptr;
        for (const Primitive& primitive : mesh.primitives) {
            drawPrimitive(primitive, skin);
        }
    }

    glPopMatrix();
}

void GltfModel::updateArmBones(const glm::quat& leftForearmQ,
                               const glm::quat& rightForearmQ,
                               const glm::quat& leftUpperArmQ,
                               const glm::quat& rightUpperArmQ)
{
    for (size_t i = 0; i < nodes.size(); ++i) {
        animatedLocalRotations[i] = nodes[i].rotation;
    }

    computeGlobals();
    applyWorldDirection(leftArm, leftUpperArmQ * glm::vec3(0.0f, -1.0f, -0.2f));
    computeGlobals();

    // ── Decouple forearm from upper-arm transport rotation ──────────
    //
    // Both sensors report ABSOLUTE world orientation. If you rotate the
    // upper arm, the forearm sensor (carried along rigidly) reports a
    // changed orientation too — even with the elbow held straight. Fed
    // directly to applyWorldDirection, this makes the forearm bone
    // visually swing whenever the upper arm moves.
    //
    // Fix: compute "elbow flexion" as the forearm sensor's orientation
    // RELATIVE to the upper-arm sensor's orientation:
    //
    //     elbowQ = inverse(upperArmQ) * forearmQ
    //
    // This cancels out the shared upper-arm transport rotation, leaving
    // only the elbow's own contribution. applyWorldDirection() then
    // expresses this direction in the forearm bone's local frame by
    // dividing out the parent (upper-arm) bone's CURRENT animated world
    // rotation — so the forearm bend tracks elbowQ correctly: it stays
    // visually fixed relative to the upper arm when only the upper arm
    // rotates, and bends only in response to genuine elbow motion.
    glm::quat leftElbowQ = glm::normalize(glm::inverse(leftUpperArmQ) * leftForearmQ);
    applyWorldDirection(leftForeArm, leftElbowQ * glm::vec3(0.0f, -1.0f, -0.2f));
    computeGlobals();

    applyWorldDirection(rightArm, rightUpperArmQ * glm::vec3(0.0f, -1.0f, -0.2f));
    computeGlobals();

    glm::quat rightElbowQ = glm::normalize(glm::inverse(rightUpperArmQ) * rightForearmQ);
    applyWorldDirection(rightForeArm, rightElbowQ * glm::vec3(0.0f, -1.0f, -0.2f));
    computeGlobals();
}

void GltfModel::updateLegBones(const glm::quat& leftThighQ,
                                const glm::quat& rightThighQ,
                                const glm::quat& leftShinQ,
                                const glm::quat& rightShinQ)
{
    applyWorldDirection(leftThigh,  leftThighQ  * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();
    applyWorldDirection(leftShin,   leftShinQ   * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();
    applyWorldDirection(rightThigh, rightThighQ * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();
    applyWorldDirection(rightShin,  rightShinQ  * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();
}

void GltfModel::updateTorsoBones(const glm::quat& hipsQ, const glm::quat& chestQ)
{
    applyWorldDirection(hips,  hipsQ  * glm::vec3(0.0f, 1.0f, 0.0f));
    computeGlobals();
    applyWorldDirection(chest, chestQ * glm::vec3(0.0f, 1.0f, 0.0f));
    computeGlobals();
}

void GltfModel::applyWorldRotation(const BoneTarget& target, const glm::quat& worldRotation)
{
    if (!target.valid) {
        return;
    }

    glm::quat parentWorld(1.0f, 0.0f, 0.0f, 0.0f);
    int parent = nodes[target.node].parent;
    if (parent >= 0) {
        parentWorld = nodeWorldRotation(parent);
    }

    animatedLocalRotations[target.node] = glm::normalize(glm::inverse(parentWorld) * worldRotation);
}

void GltfModel::applyWorldDirection(const BoneTarget& target, const glm::vec3& worldDirection)
{
    if (!target.valid || glm::length(worldDirection) < 1e-6f) {
        return;
    }

    glm::quat parentWorld(1.0f, 0.0f, 0.0f, 0.0f);
    int parent = nodes[target.node].parent;
    if (parent >= 0) {
        parentWorld = nodeWorldRotation(parent);
    }

    glm::vec3 childLocal = nodes[target.child].translation;
    if (glm::length(childLocal) < 1e-6f) {
        return;
    }

    glm::vec3 currentDirection = glm::normalize(nodes[target.node].rotation * glm::normalize(childLocal));
    glm::vec3 targetDirection = glm::normalize(glm::inverse(parentWorld) * glm::normalize(worldDirection));
    glm::quat delta = rotationBetween(currentDirection, targetDirection);
    animatedLocalRotations[target.node] = glm::normalize(delta * nodes[target.node].rotation);
}

void GltfModel::prepareBoneTarget(BoneTarget& target, const char* nodeName, const char* childName)
{
    target.node = findNode(nodeName);
    target.child = findNode(childName);
    target.valid = target.node >= 0 && target.child >= 0;
    if (!target.valid) {
        std::cerr << "Missing GLB bone target: " << nodeName << " -> " << childName << "\n";
        return;
    }

    target.alignment = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
}

int GltfModel::findNode(const std::string& name) const
{
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].name == name) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

glm::vec3 GltfModel::nodeWorldPosition(int nodeIndex) const
{
    glm::vec4 p = bindGlobals[nodeIndex] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    return glm::vec3(p) / p.w;
}

glm::quat GltfModel::nodeWorldRotation(int nodeIndex) const
{
    glm::mat3 basis(animatedGlobals[nodeIndex]);
    basis[0] = glm::normalize(basis[0]);
    basis[1] = glm::normalize(basis[1]);
    basis[2] = glm::normalize(basis[2]);
    return glm::normalize(glm::quat_cast(basis));
}

glm::quat GltfModel::rotationBetween(glm::vec3 from, glm::vec3 to) const
{
    from = glm::normalize(from);
    to = glm::normalize(to);
    float cosTheta = glm::clamp(glm::dot(from, to), -1.0f, 1.0f);
    if (cosTheta > 0.9999f) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    if (cosTheta < -0.9999f) {
        glm::vec3 axis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), from);
        if (glm::length(axis) < 1e-5f) {
            axis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), from);
        }
        return glm::angleAxis(glm::radians(180.0f), glm::normalize(axis));
    }
    glm::vec3 axis = glm::normalize(glm::cross(from, to));
    return glm::angleAxis(std::acos(cosTheta), axis);
}

glm::mat4 GltfModel::skinMatrix(const Skin& skin, int jointSlot) const
{
    if (jointSlot < 0 || jointSlot >= static_cast<int>(skin.joints.size())) {
        return glm::mat4(1.0f);
    }
    int nodeIndex = skin.joints[jointSlot];
    return animatedGlobals[nodeIndex] * skin.inverseBindMatrices[jointSlot];
}

void GltfModel::drawPrimitive(const Primitive& primitive, const Skin* skin)
{
    glm::vec4 c = primitive.color;
    glColor4f(c.r, c.g, c.b, c.a);
    glBegin(GL_TRIANGLES);

    for (unsigned int index : primitive.indices) {
        if (index >= primitive.vertices.size()) {
            continue;
        }
        const Vertex& vertex = primitive.vertices[index];
        glm::vec4 skinnedPosition(0.0f);
        glm::vec3 skinnedNormal(0.0f);

        if (skin && glm::dot(vertex.weights, glm::vec4(1.0f)) > 0.0f) {
            for (int i = 0; i < 4; ++i) {
                float weight = vertex.weights[i];
                if (weight <= 0.0f) {
                    continue;
                }
                glm::mat4 m = skinMatrix(*skin, vertex.joints[i]);
                skinnedPosition += weight * (m * glm::vec4(vertex.position, 1.0f));
                skinnedNormal += weight * glm::mat3(m) * vertex.normal;
            }
        } else {
            skinnedPosition = glm::vec4(vertex.position, 1.0f);
            skinnedNormal = vertex.normal;
        }

        if (glm::length(skinnedNormal) > 1e-6f) {
            skinnedNormal = glm::normalize(skinnedNormal);
        } else {
            skinnedNormal = glm::vec3(0.0f, 1.0f, 0.0f);
        }

        glNormal3fv(glm::value_ptr(skinnedNormal));
        glVertex3fv(glm::value_ptr(glm::vec3(skinnedPosition)));
    }

    glEnd();
}
