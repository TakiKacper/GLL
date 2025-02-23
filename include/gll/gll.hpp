/*
This library depends on following libraries:
stb/stb_image   - https://github.com/nothings/stb/blob/master/stb_image.h
assimp          - https://github.com/assimp/assimp
*/

#pragma once

#include <stddef.h>
#include <cstdint>

#include <vector>
#include <array>
#include <map>

namespace gll
{
    template<class T>
    using result = std::pair<bool, T>;

    struct image
    {
        size_t      width;
        size_t      height;
        uint8_t     color_channels;
        
        void*       pixel_data;
        size_t      pixel_data_size;
    };

    //jpeg, png, tga, bmp, psd, gif, hdr, pic, pnm
    result<image> load_image(const char* filepath);
    void free_image(image& img);

    struct mesh
    {
        bool has_normals;
        bool has_tex_coords;
        bool has_tangents_and_bitangents;
        bool has_bones;

        //if interleaved, positions contains all the informations in order of declarations

        std::vector<float>  positions;
        std::vector<float>  normals;
        std::vector<float>  texcoords;
        std::vector<float>  tangents;
        std::vector<float>  bitangents;
        std::vector<int>    bones_ids;
        std::vector<float>  bones_weights;

        std::vector<unsigned int>   indicies_data;

        int material_id;
    };

    struct bone_info
    {
        int id;

        std::array<float, 4> offset_matrix_row_0;
        std::array<float, 4> offset_matrix_row_1;
        std::array<float, 4> offset_matrix_row_2;
        std::array<float, 4> offset_matrix_row_3;
    };

    struct model
    {
        std::map<std::string, bone_info> bones;
        std::vector<mesh> meshes;
    };

    result<model> load_model(const char* filepath, bool interleave_attributes, int max_influencial_bones);
    void free_model(model& mod);
}

#ifndef GLL_IMPLEMENTATION

using namespace gll;

#include "stb/stb_image.hpp"

result<image> gll::load_image(const char* filepath)
{
    stbi_set_flip_vertically_on_load(true);

    image img;
    int width, height, channels;

    unsigned char* data = stbi_load(
        filepath, 
        &width, 
        &height,
        &channels,
        0
    );

    if (!data)
        return {false, {}};

    img.width = width;
    img.height = height;
    img.color_channels = channels;

    img.pixel_data = data;
    img.pixel_data_size = (img.width * img.height * img.color_channels * sizeof(float));

    return {true, std::move(img)};
}

void gll::free_image(image& img)
{
    stbi_image_free(img.pixel_data);
}

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

void process_assimp_mesh(
    model&          output, 
    const bool      interleave, 
    int             max_influencial_bones,
    aiMesh*         mesh, 
    const aiScene*  scene
)
{
    auto continue_on_mesh = [&]() -> gll::mesh&
    {
        for (auto& m : output.meshes)
            if (
                m.material_id == mesh->mMaterialIndex &&
                m.has_normals == mesh->HasBones() &&
                m.has_bones   == mesh->HasBones() &&
                m.has_tangents_and_bitangents == mesh->HasTangentsAndBitangents() &&
                m.has_tex_coords == mesh->HasTextureCoords(0)
            )
                return m;
            
        output.meshes.push_back({});
        return output.meshes.back();
    };

    gll::mesh& outmesh = continue_on_mesh();

    auto& positions_target      = outmesh.positions;
    auto& normals_target        = interleave ? outmesh.positions : outmesh.normals;
    auto& texcoords_target      = interleave ? outmesh.positions : outmesh.texcoords;
    auto& tangents_target       = interleave ? outmesh.positions : outmesh.tangents;
    auto& bitangents_target     = interleave ? outmesh.positions : outmesh.bitangents;
    //auto& bones_ids_target      = interleave ? outmesh.positions : outmesh.bones_ids;     missmatch types
    auto& bones_weights_target  = interleave ? outmesh.positions : outmesh.bones_weights;

    bool positions  = mesh->HasPositions();
    bool normals    = mesh->HasNormals();
    bool texcoords  = mesh->HasTextureCoords(0);
    bool tangents   = mesh->HasTangentsAndBitangents();
    bool bones      = mesh->HasBones();

    outmesh.has_normals = normals;
    outmesh.has_tex_coords = texcoords;
    outmesh.has_tangents_and_bitangents = tangents;
    outmesh.has_bones = bones; 

    int elements = (
        positions * 3 + 
        normals * 3 + 
        texcoords * 2 + 
        tangents * 6 + 
        bones * 2 * max_influencial_bones
    );

    size_t inherited_vertices = interleave ? positions_target.size() / elements : positions_target.size() / 3;
    size_t veritces_count = mesh->mNumVertices;

    if (interleave)
    {
        positions_target.reserve(
            positions_target.size() +
            veritces_count * (
                positions * sizeof(float) * 3 +
                normals   * sizeof(float) * 3 +
                texcoords * sizeof(float) * 2 +
                tangents  * sizeof(float) * 6 +
                bones     * (sizeof(int)  * max_influencial_bones + sizeof(float) * max_influencial_bones)
            )
        );
    }
    else
    {
        positions_target.reserve(positions_target.size() + positions * sizeof(float) * 3);
        normals_target.reserve(normals_target.size() + normals * sizeof(float) * 3);
        texcoords_target.reserve(texcoords_target.size() + texcoords * sizeof(float) * 2);

        tangents_target.reserve(tangents_target.size() + texcoords * sizeof(float) * 3);
        bitangents_target.reserve(bitangents_target.size() + tangents * sizeof(float) * 3);

        outmesh.bones_ids.reserve(outmesh.bones_ids.size() + bones * sizeof(int) * max_influencial_bones);
        bones_weights_target.reserve(bones_weights_target.size() + bones * sizeof(float) * max_influencial_bones);
    }
    
    for(unsigned int i = 0; i < veritces_count; i++)
    {
        if (positions)
        {
            positions_target.push_back(mesh->mVertices[i].x);
            positions_target.push_back(mesh->mVertices[i].z);
            positions_target.push_back(mesh->mVertices[i].y);
        }

        if (normals)
        {
            normals_target.push_back(mesh->mNormals[i].x);
            normals_target.push_back(mesh->mNormals[i].z);
            normals_target.push_back(mesh->mNormals[i].y);
        }

        if (texcoords)
        {
            texcoords_target.push_back(mesh->mTextureCoords[0][i].x);
            texcoords_target.push_back(mesh->mTextureCoords[0][i].y);
        }
        
        if (tangents)
        {
            tangents_target.push_back(mesh->mTangents[i].x);
            tangents_target.push_back(mesh->mTangents[i].z);
            tangents_target.push_back(mesh->mTangents[i].y);

            bitangents_target.push_back(mesh->mBitangents[i].x);
            bitangents_target.push_back(mesh->mBitangents[i].z);
            bitangents_target.push_back(mesh->mBitangents[i].y);
        }

        if (bones)
        {
            if (interleave)
            {
                for (int i = 0; i < max_influencial_bones; i++)
                {
                    union {
                        float f;
                        int i;
                    } x;

                    x.i = -1;
                    outmesh.positions.push_back(x.f);
                }
            }
            else
            {
                for (int i = 0; i < max_influencial_bones; i++)
                    outmesh.bones_ids.push_back(-1);
            }

            bones_weights_target.resize(bones_weights_target.size() + max_influencial_bones);
        }
    }

    for (unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        auto face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++)
            outmesh.indicies_data.push_back(face.mIndices[j]);
    }

    if (bones)
    {
		for (int bone_id = 0; bone_id < mesh->mNumBones; ++bone_id)
		{
			std::string name = mesh->mBones[bone_id]->mName.C_Str();

			if (output.bones.find(name) == output.bones.end())
			{
                auto& mat = mesh->mBones[bone_id]->mOffsetMatrix;

				bone_info info;
				info.id = output.bones.size();

                info.offset_matrix_row_0 = {mat.a1, mat.b1, mat.c1, mat.d1};    
                info.offset_matrix_row_1 = {mat.a2, mat.b2, mat.c2, mat.d2};
                info.offset_matrix_row_2 = {mat.a3, mat.b3, mat.c3, mat.d3};
                info.offset_matrix_row_3 = {mat.a4, mat.b4, mat.c4, mat.d4};     

				output.bones.insert({std::move(name), std::move(info)});
			}

			auto weights = mesh->mBones[bone_id]->mWeights;
			int  weights_count = mesh->mBones[bone_id]->mNumWeights;

			for (int weight_index = 0; weight_index < weights_count; ++weight_index)
			{
                //index within this assimp mesh
				int   vertex_id  = inherited_vertices + weights[weight_index].mVertexId;
				float weight     = weights[weight_index].mWeight;

                for (int i = 0; i < max_influencial_bones; ++i)
                {
                    if (interleave)
                    {
                        //jump to vertex begin
                        int p = vertex_id * elements;

                        //skip following
                        p += positions * 3;
                        p += normals * 3;
                        p += texcoords * 2;
                        p += tangents * 6;

                        //go to i bone
                        p += i;

                        union {
                            float f;
                            int i;
                        } x;

                        x.f = positions_target[p];
                        
                        if (x.i < 0)
                        {
                            x.i = output.bones.size() - 1;
                            positions_target[p] = x.f;
                            //move to corresponding weight
                            i += max_influencial_bones;
                            positions_target[p] = weight;
                            break;
                        }
                    }
                    else
                    {
                        int p = vertex_id * max_influencial_bones;
                        
                        if (outmesh.bones_ids[p + i] < 0)
                        {
                            outmesh.bones_ids[p + 1] = output.bones.size() - 1;
                            outmesh.bones_weights[p + 1] = weight;
                            break;
                        }
                    }
                }
			}
		}
	}
}

void process_assimp_node(
    model&          output, 
    const bool      interleave,
    int             max_influencial_bones,
    aiNode*         node,
    const aiScene*  scene
)
{
    for(unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        process_assimp_mesh(output, interleave, max_influencial_bones, mesh, scene);
    }
    
    for(unsigned int i = 0; i < node->mNumChildren; i++)
        process_assimp_node(output, interleave, max_influencial_bones, node->mChildren[i], scene);
}

result<model> gll::load_model(const char* filepath, bool interleave_attributes, int max_influencial_bones)
{
    model output;

    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(filepath, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);	
	
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) 
        return {false, {}};

    process_assimp_node(output, interleave_attributes, max_influencial_bones, scene->mRootNode, scene);
    return {true, std::move(output)};
}

void gll::free_model(model& mod)
{

}

#endif
