/*******************************************************************************************
 * File:        kdtree.cpp
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     05/19/2022
 * 
 * Description: kdtree is used to querry closest map points
*******************************************************************************************/

#include "kdtree.h"
#include <queue>

namespace maplite{
    
    KdNode::KdNode()
    {
        parent_ = nullptr;
        left_ = nullptr;
        right_ = nullptr;
        is_leaf_ = false;
    }

    KdNode::~KdNode()
    {

    }

    KdTree::KdTree()
    {
        head_ = nullptr;
        maxv_pos_ = -1;
        size_ = 0;
    }


    KdTree::~KdTree()
    {
        FreeNode(head_);
        if(head_ == nullptr){
            size_ = 0;
        }
    }

    bool KdTree::Create(const std::vector<double>& bias, const std::vector<coordinate_t>& point_cloud)
    {
        size_ = point_cloud.size();
        head_ = BuildKDTree(bias, point_cloud);

        return true;
    }

    KdNode* KdTree::BuildKDTree(const std::vector<double>& bias, const std::vector<coordinate_t>& point_cloud)
    {
        if (point_cloud.empty())
        {
            return nullptr;
        }
        
        //get variance for xyz, and get max
        maxv_pos_ = MaxVarianceD(bias, point_cloud);
        if (maxv_pos_ < 0)
        {
            return nullptr;
        }
        

        // Sort from small to large on the dimension with the largest variance, and find the median
        std::vector<coordinate_t>& m_point_cloud = const_cast<std::vector<coordinate_t>&>(point_cloud);
        std::sort(m_point_cloud.begin(), m_point_cloud.end(), [this](coordinate_t& left, coordinate_t& right){
            return left.val_[maxv_pos_] < right.val_[maxv_pos_];
        });

        size_t mid_idx = m_point_cloud.size() / 2;

        KdNode* pNode = new KdNode();
        pNode->point_ = m_point_cloud[mid_idx];

        //Divide the left and right subspaces, excluding the median point
        std::vector<coordinate_t> left_point_cloud, right_point_cloud;
        left_point_cloud.insert(left_point_cloud.end(), m_point_cloud.begin(), m_point_cloud.begin() + mid_idx);
        right_point_cloud.insert(right_point_cloud.end(), m_point_cloud.begin() + mid_idx + 1, m_point_cloud.end());

        pNode->left_ = BuildKDTree(bias, left_point_cloud);
        pNode->right_ = BuildKDTree(bias, right_point_cloud);

        //set parent node
        if (pNode->left_ != nullptr)
        {
            pNode->left_->parent_ = pNode;
        }

        if (pNode->right_ != nullptr)
        {
            pNode->right_->parent_ = pNode;
        }

        //judge if leaf node
        if (pNode->left_ == nullptr && pNode->right_ == nullptr)
        {
            pNode->is_leaf_ = true;
        }
        return pNode;
    }

    void KdTree::PrintKDTree()
    {
        std::queue<std::pair<KdNode*, int> > nodes;
        nodes.push(std::pair<KdNode*, int>(head_, 0));

        int last_layer = 0;
        while (!nodes.empty())
        {
            auto data = nodes.front();
            if (data.first != nullptr)
            {
                if (data.second > last_layer)
                {
                    last_layer = data.second;
                    std::cout << std::endl;
                }

                if (data.second < last_layer)
                {   //left and right node of a parent node
                    std::cout << data.first->point_.val_[0] << "," << data.first->point_.val_[1] << "\t";
                }
                else
                {
                    std::cout << data.first->point_.val_[0] << "," << data.first->point_.val_[1] << " ";
                }

                nodes.push(std::pair<KdNode*, int>(data.first->left_, data.second + 1));
                nodes.push(std::pair<KdNode*, int>(data.first->right_, data.second));
            }
            nodes.pop();
        }
    }

    bool KdTree::FindKNearestByR(const coordinate_t& point, std::vector<coordinate_t>& npointes, double radius, int k, double range)
    {
        //closest point
        std::vector<KdNode*> cPoints;
        cPoints.reserve(size_);

        //search leaf points for current closest point
        KdNode* curr_node = FindLeafNode(head_, point);
        if (curr_node == nullptr)
        {
            std::cout<<"!!!!!!curr_node == nullptr, return and switch!!!!!!"<<std::endl;
            return true;
        }
        
        // std::cout << "---point: " << point.val_[0] << ", " << point.val_[1] << "---Dist: " << Dist(point, curr_node->point_)<< "---Leaf: " << curr_node->point_.val_[0] << ", " << curr_node->point_.val_[1] << std::endl;

        if (Dist(point, curr_node->point_) <= range)
        {
            cPoints.push_back(curr_node);
        }

        FindPCNode(curr_node, point, cPoints, range);

        std::sort(cPoints.begin(), cPoints.end(), [this, point](KdNode*& left, KdNode*& right){
            return Dist(point, left->point_) < Dist(point, right->point_);
        });
        
        int num, l;
        if (radius > 0){
            l = 0;
            while(Dist(point, cPoints[l]->point_) <= radius && l < (int)(cPoints.size())){
                l++;
            } // o too small, num too big
            if (k > 0){
                k = k < (int)(cPoints.size())? k : (int)(cPoints.size());
                num =  k < l? k : l;
            }
            else{
                num = l;
            }
        } 
        else{
            if (k > 0){
                num = k < (int)(cPoints.size())? k : (int)(cPoints.size());
            }
            else{
                num = cPoints.size();
            }
        }
        

        npointes.resize(num);

        for (int i = 0; i < num; i++)
        {
            npointes[i] = cPoints[i]->point_;
        }

        return true;
    }

    KdNode* KdTree::FindLeafNode(KdNode* pNode, const coordinate_t& point)
    {
        if (pNode == nullptr)
        {
            return nullptr;
        }

        if (pNode->is_leaf_)
        {
            return pNode;
        }

        if (point.val_[maxv_pos_] < pNode->point_.val_[maxv_pos_])
        {
            return FindLeafNode(pNode->left_, point);
        }
        else
        {
            auto ret = FindLeafNode(pNode->right_, point);
            if (ret == nullptr && point.val_[maxv_pos_] == pNode->point_.val_[maxv_pos_])
            {
                ret = pNode;
            }

            return ret;
        }
    }

    void KdTree::FindPCNode(KdNode* pNode, const coordinate_t& point, std::vector<KdNode*>& cPoints, double range)
    {
        //root node
        if (pNode->parent_ == nullptr)
        {
            return;
        }

        KdNode* curr_node = pNode->parent_;

        if (Dist(point, curr_node->point_) <= range)
        {
            cPoints.push_back(curr_node);
        }

        //Determine whether another node splitting plane of the node's parent node intersects the sphere
        KdNode* oth_node = curr_node->left_;
        if (oth_node == pNode)
        {
            oth_node = curr_node->right_;
        }

        if (oth_node != nullptr)
        {
            double dist = std::abs(point.val_[maxv_pos_] - oth_node->point_.val_[maxv_pos_]);
            if (dist <= range)
            {   //search downward iteratively
                FindCNode(oth_node, point, cPoints, range);
            }
        }

        //search upward iteratively
        FindPCNode(curr_node, point, cPoints, range);
    }

    void KdTree::FindCNode(KdNode* pNode, const coordinate_t& point, std::vector<KdNode*>& cPoints, double range)
    {
        if (Dist(point, pNode->point_) <= range)
        {
            cPoints.push_back(pNode);
        }

        if (pNode->left_ != nullptr)
        {
            double dist = std::abs(point.val_[maxv_pos_] - pNode->left_->point_.val_[maxv_pos_]);
            if (dist <= range)
            {
                FindCNode(pNode->left_, point, cPoints, range);
            }
        }

        if (pNode->right_ != nullptr)
        {
            double dist = std::abs(point.val_[maxv_pos_] - pNode->right_->point_.val_[maxv_pos_]);
            if (dist <= range)
            {
                FindCNode(pNode->right_, point, cPoints, range);
            }
        }
    }

    int KdTree::MaxVarianceD(const std::vector<double>& bias, const std::vector<coordinate_t>& point_cloud)
    {
        if (point_cloud.empty())
        {
            return -1;
        }

        double sum[DIM] = { 0 }, sq_sum[DIM] = { 0 }, mean_v[DIM] = { 0 };
        std::vector<double> variance(DIM, 0);

        for (int i = 0; i < DIM; i++)
        {
            for (size_t j = 0; j < point_cloud.size(); j++)
            {
                sum[i] += point_cloud[j].val_[i];
                sq_sum[i] += pow(point_cloud[j].val_[i] - bias[i], 2);
            }

            mean_v[i] = sum[i] / point_cloud.size() - bias[i];
            variance[i] = sq_sum[i] / point_cloud.size() - pow(mean_v[i], 2);
        }

        auto maxv_pos = std::max_element(variance.begin(), variance.end());

        return maxv_pos - variance.begin();
    }

    double KdTree::Dist(const coordinate_t& point1, const coordinate_t& point2)
    {
        return sqrt(pow(point1.val_[0] - point2.val_[0], 2) + pow(point1.val_[1] - point2.val_[1], 2));
    }

    void KdTree::FreeNode(KdNode*& pNode)
    {
        if(pNode != nullptr)
        {
            FreeNode(pNode->left_);
            FreeNode(pNode->right_);

            delete pNode;
            pNode = nullptr;
        }
    }
}