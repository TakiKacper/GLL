/*
This library depends on following libraries:
stb/stb_image   - https://github.com/nothings/stb/blob/master/stb_image.h
assimp          - https://github.com/assimp/assimp
*/

#pragma once

#include <stddef.h>
#include <cstdint>

#include <list>
#include <vector>
#include <set>
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

    struct image_load_settings
    {
        bool flip_vertically    = true;
    };

    //jpeg, png, tga, bmp, psd, gif, hdr, pic, pnm
    result<image> load_image(const char* filepath, const image_load_settings& settings);
    void free_image(image& img);

    struct model
    {
        enum class attribute
        {
            position            = 0, 
            normal              = 1, 
            texcoord            = 2,
            tangents_bitangents = 3,
            bones_indices       = 4,
            bones_weights       = 5
        };

        struct mesh
        {
            std::set<attribute>             attributes;
            std::list<std::vector<float>>   vertices;
            std::vector<unsigned int>       indicies;
            int                             material_id;
        };

        struct bone_info
        {
            int id;
            std::array<float, 4> offset_matrix_row_0;
            std::array<float, 4> offset_matrix_row_1;
            std::array<float, 4> offset_matrix_row_2;
            std::array<float, 4> offset_matrix_row_3;
        };

        std::map<std::string, bone_info>    bones;
        std::vector<mesh>                   meshes;
    };

    struct model_load_settings
    {
        bool                        interleave_attributes = true;
        int                         max_influencial_bones = 4;
        std::set<model::attribute>  force_attributes;
    };

    result<model> load_model(const char* filepath, const model_load_settings& settings);
    void free_model(model& mod);
}

#ifndef GLL_IMPLEMENTATION

using namespace gll;

#include "stb/stb_image.hpp"

result<image> gll::load_image(const char* filepath, const image_load_settings& settings)
{
    stbi_set_flip_vertically_on_load_thread(settings.flip_vertically);

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

void process_assimp_vertex_attrib(
    gll::model::mesh&           output,
    size_t                      vertex_id,
    gll::model::attribute       attrib,
    aiMesh*                     mesh,
    std::vector<float>*         target,
    const model_load_settings&  settings
)
{
    switch (attrib)
    {
    case model::attribute::position:
        if (!mesh->HasPositions())              goto _process_assimp_vertex_attrib_push_zeros;
        target->push_back(mesh->mVertices[vertex_id].x);
        target->push_back(mesh->mVertices[vertex_id].z);
        target->push_back(mesh->mVertices[vertex_id].y);
        break;
    case model::attribute::normal:
        if (!mesh->HasNormals())                goto _process_assimp_vertex_attrib_push_zeros;
        target->push_back(mesh->mNormals[vertex_id].x);
        target->push_back(mesh->mNormals[vertex_id].z);
        target->push_back(mesh->mNormals[vertex_id].y);
        break;
    case model::attribute::texcoord:
        if (!mesh->HasTextureCoords(0))         goto _process_assimp_vertex_attrib_push_zeros;
        target->push_back(mesh->mTextureCoords[0][vertex_id].x);
        target->push_back(mesh->mTextureCoords[0][vertex_id].y);
        break;
    case model::attribute::tangents_bitangents:
        if (!mesh->HasTangentsAndBitangents())  goto _process_assimp_vertex_attrib_push_zeros;
        target->push_back(mesh->mTangents[vertex_id].x);
        target->push_back(mesh->mTangents[vertex_id].z);
        target->push_back(mesh->mTangents[vertex_id].y);
        target->push_back(mesh->mBitangents[vertex_id].x);
        target->push_back(mesh->mBitangents[vertex_id].z);
        target->push_back(mesh->mBitangents[vertex_id].y);
        break;
    case model::attribute::bones_indices:
        union {
            float f;
            int i;
        } conversion;
        conversion.i = -1;

        for (int i = 0; i < settings.max_influencial_bones; i++)
            target->push_back(conversion.f);
        break;
    case model::attribute::bones_weights:
        for (int i = 0; i < settings.max_influencial_bones; i++)
            target->push_back(0);
        break;
    }
    return;
    
_process_assimp_vertex_attrib_push_zeros:
    size_t count;
    
    if (attrib == model::attribute::texcoord)                   count = 2;
    else if (attrib == model::attribute::tangents_bitangents)   count = 6;
    else                                                        count = 3;
                    
    for (int i = 0; i < count; i++)
        target->push_back(0);
    return;
}

void process_assimp_mesh(
    model&                      output, 
    const model_load_settings&  settings,
    aiMesh*                     mesh
)
{
    //Create mesh

    output.meshes.push_back({});
    auto& outmesh = output.meshes.back();
    outmesh.material_id = mesh->mMaterialIndex;

    //Load Indicies
    
    for (unsigned int face_id = 0; face_id < mesh->mNumFaces; face_id++)
    {
        auto face = mesh->mFaces[face_id];
        for (unsigned int j = 0; j < face.mNumIndices; j++)
            outmesh.indicies.push_back(face.mIndices[j]);
    }

    //Findout vertex layout

    std::set<gll::model::attribute> model_attribs;
    if (mesh->HasPositions())               model_attribs.insert(gll::model::attribute::position);
    if (mesh->HasNormals())                 model_attribs.insert(gll::model::attribute::normal);
    if (mesh->HasTextureCoords(0))          model_attribs.insert(gll::model::attribute::texcoord);
    if (mesh->HasTangentsAndBitangents())   model_attribs.insert(gll::model::attribute::tangents_bitangents);
    if (mesh->HasBones()) {
        model_attribs.insert(gll::model::attribute::bones_indices);
        model_attribs.insert(gll::model::attribute::bones_weights);
    };

    model_attribs.insert(
        settings.force_attributes.begin(),
        settings.force_attributes.end()
    ); 

    //Create containers for vertices

    const size_t vertices_count = mesh->mNumVertices;
    std::map<gll::model::attribute, std::vector<float>*> attribs_save_targets_map; 

    auto elements_per_attrib = [&](gll::model::attribute attrib){
        switch (attrib)
        {
        case gll::model::attribute::position:              return 3;
        case gll::model::attribute::normal:                return 3;
        case gll::model::attribute::texcoord:              return 2;
        case gll::model::attribute::tangents_bitangents:   return 6;
        case gll::model::attribute::bones_indices:         return settings.max_influencial_bones;
        case gll::model::attribute::bones_weights:         return settings.max_influencial_bones;
        }
        return 0;
    };

    if (settings.interleave_attributes)
    {
        outmesh.vertices.push_back({});
        auto& target = outmesh.vertices.back();
        
        size_t vertex_length = 0;
        for (auto& attrib : model_attribs)
        {
            attribs_save_targets_map.insert({attrib, &target});
            vertex_length += elements_per_attrib(attrib);
        }
            
        target.reserve(vertex_length * vertices_count);
    }
    else
    {
        for (auto& attrib : model_attribs)
        {
            outmesh.vertices.push_back({});
            auto& target = outmesh.vertices.back();
            target.reserve(vertices_count * elements_per_attrib(attrib));
            attribs_save_targets_map.insert({attrib, &target});
        }
    }

    outmesh.attributes = model_attribs;

    //Load Vertices

    for (size_t vertex_id = 0; vertex_id < vertices_count; vertex_id++)
    {
        for (auto& attrib : model_attribs)
        {
            process_assimp_vertex_attrib(
                outmesh,
                vertex_id,
                attrib,
                mesh,
                attribs_save_targets_map[attrib],
                settings
            );
        }
    }
}

void process_assimp_node(
    model&          output, 
    const model_load_settings& settings,
    aiNode*         node,
    const aiScene*  scene
)
{
    for(unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        process_assimp_mesh(output, settings, mesh);
    }
    
    for(unsigned int i = 0; i < node->mNumChildren; i++)
        process_assimp_node(output, settings, node->mChildren[i], scene);
}

result<model> gll::load_model(const char* filepath, const model_load_settings& settings)
{
    model output;

    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(filepath, aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs | aiProcess_CalcTangentSpace);	
	
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) 
        return {false, {}};

    process_assimp_node(output, settings, scene->mRootNode, scene);
    return {true, std::move(output)};
}

void gll::free_model(model& mod)
{

}

#endif
