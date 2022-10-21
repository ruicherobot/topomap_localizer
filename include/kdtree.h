/*******************************************************************************************
 * File:        kdtree.h
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     05/19/2022
 * 
 * Description: kdtree is used to sort map points or bus stops
*******************************************************************************************/
#ifndef KDTREE_H
#define KDTREE_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "coordinate_t.h"

namespace maplite {

    class KdNode
    {
    public:
        KdNode();
        ~KdNode();

        coordinate_t point_;
        KdNode* parent_;
        KdNode* left_;
        KdNode* right_;
        bool is_leaf_;
    };

    class KdTree
    {
    public:
        KdTree();
        ~KdTree();

        //construct kdtree
        bool Create(const std::vector<double>& bias, const std::vector<coordinate_t>& point_cloud);
        KdNode* BuildKDTree(const std::vector<double>& bias, const std::vector<coordinate_t>& point_cloud);
        
        //closest k points within certain range，if k<0，all points
        bool FindKNearestByR(const coordinate_t& point, std::vector<coordinate_t>& npointes, double radius, int k, double range);
        void PrintKDTree();

        virtual int Insert(const coordinate_t& point){ return 0; }
        virtual bool Delete(int idx){ return true; }

        KdNode* FindLeafNode(KdNode* pnode, const coordinate_t& point);
        void FindPCNode(KdNode* pnode, const coordinate_t& point, std::vector<KdNode*>& mnpointes, double range);
        void FindCNode(KdNode* pnode, const coordinate_t& point, std::vector<KdNode*>& mnpointes, double range);

        int MaxVarianceD(const std::vector<double>& bias, const std::vector<coordinate_t>& point_cloud);
        double Dist(const coordinate_t& point1, const coordinate_t& point2);

        void FreeNode(KdNode*& pnode);

        KdNode* head_;

        int maxv_pos_;

        int size_;
    };

}
#endif /* KDTREE_H */
