/**
 * @file multi_node_tree.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __MULTI_NODE_TREE_HPP__
#define __MULTI_NODE_TREE_HPP__

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <memory>

template<typename T>
class TreeNode : public std::enable_shared_from_this<TreeNode<T>>
{
public:
    T data;
    std::vector<std::shared_ptr<TreeNode<T>>> children;
    std::weak_ptr<TreeNode<T>> parent;

    // Constructor
    explicit TreeNode(const T& value)
        : data(value)
    {
    }

    // Add child node
    std::shared_ptr<TreeNode<T>> addChild(const T& value)
    {
        auto child = std::make_shared<TreeNode<T>>(value);
        child->parent = this->shared_from_this();
        children.push_back(child);
        return child;
    }

    // Remove child by value
    bool removeChild(const T& value)
    {
        auto it = std::find_if(children.begin(), children.end(),
            [&value](const std::shared_ptr<TreeNode<T>>& node)
            {
                return node->data == value;
            });
        
        if (it != children.end())
        {
            children.erase(it);
            return true;
        }
        return false;
    }

    // Find child by value
    std::shared_ptr<TreeNode<T>> findChild(const T& value)
    {
        auto it = std::find_if(children.begin(), children.end(),
            [&value](const std::shared_ptr<TreeNode<T>>& node)
            {
                return node->data == value;
            });
        return (it != children.end()) ? *it : nullptr;
    }

    // Check if node is leaf
    bool isLeaf() const
    {
        return children.empty();
    }

    // Get number of children
    size_t getChildCount() const
    {
        return children.size();
    }

private:
    void dfsHelper(const std::shared_ptr<TreeNode<T>>& node, 
                   std::function<void(const T&)> visitFunc) const
    {
        if (node)
        {
            visitFunc(node->data);
            for (const auto& child : node->children)
            {
                dfsHelper(child, visitFunc);
            }
        }
    }

    std::shared_ptr<TreeNode<T>> findHelper(const std::shared_ptr<TreeNode<T>>& node, 
                                           const T& value) const
    {
        if (!node) return nullptr;
        
        if (node->data == value)
        {
            return node;
        }

        for (const auto& child : node->children)
        {
            auto result = findHelper(child, value);
            if (result)
            {
                return result;
            }
        }
        return nullptr;
    }

    int getHeightHelper(const std::shared_ptr<TreeNode<T>>& node) const
    {
        if (!node || node->isLeaf())
        {
            return 0;
        }

        int maxHeight = 0;
        for (const auto& child : node->children)
        {
            maxHeight = std::max(maxHeight, getHeightHelper(child));
        }
        return maxHeight + 1;
    }

    size_t getSizeHelper(const std::shared_ptr<TreeNode<T>>& node) const
    {
        if (!node) return 0;
        
        size_t count = 1; // Count current node
        for (const auto& child : node->children)
        {
            count += getSizeHelper(child);
        }
        return count;
    }

    void getLeafNodesHelper(const std::shared_ptr<TreeNode<T>>& node, 
                           std::vector<T>& leaves) const
    {
        if (node->isLeaf())
        {
            leaves.push_back(node->data);
        }
        else
        {
            for (const auto& child : node->children)
            {
                getLeafNodesHelper(child, leaves);
            }
        }
    }

    void printTreeHelper(const std::shared_ptr<TreeNode<T>>& node, int level) const
    {
        if (!node) return;

        for (int i = 0; i < level; ++i)
        {
            std::cout << "  ";
        }
        std::cout << node->data << std::endl;

        for (const auto& child : node->children)
        {
            printTreeHelper(child, level + 1);
        }
    }

    bool getPathToHelper(const std::shared_ptr<TreeNode<T>>& node, 
                        const T& value, std::vector<T>& path) const
    {
        if (!node) return false;

        path.push_back(node->data);

        if (node->data == value)
        {
            return true;
        }

        for (const auto& child : node->children)
        {
            if (getPathToHelper(child, value, path))
            {
                return true;
            }
        }

        path.pop_back();
        return false;
    }

    void getNodesAtLevelHelper(const std::shared_ptr<TreeNode<T>>& node, 
                              int currentLevel, int targetLevel, 
                              std::vector<T>& result) const
    {
        if (!node) return;

        if (currentLevel == targetLevel)
        {
            result.push_back(node->data);
            return;
        }

        if (currentLevel < targetLevel)
        {
            for (const auto& child : node->children)
            {
                getNodesAtLevelHelper(child, currentLevel + 1, targetLevel, result);
            }
        }
    }

    void postOrderHelper(const std::shared_ptr<TreeNode<T>>& node, 
                        std::function<void(const T&)> visitFunc) const
    {
        if (node)
        {
            for (const auto& child : node->children)
            {
                postOrderHelper(child, visitFunc);
            }
            visitFunc(node->data);
        }
    }
};

template<typename T>
class MultiNodeTree
{
public:
    // Constructor
    MultiNodeTree()
        : root_(nullptr)
    {
    }

    // Constructor with root value
    explicit MultiNodeTree(const T& rootValue)
    {
        root_ = std::make_shared<TreeNode<T>>(rootValue);
    }

    // Set root
    void setRoot(const T& value)
    {
        root_ = std::make_shared<TreeNode<T>>(value);
    }

    // Get root
    std::shared_ptr<TreeNode<T>> getRoot() const
    {
        return root_;
    }

    // Check if tree is empty
    bool isEmpty() const
    {
        return root_ == nullptr;
    }

    // Clear the tree
    void clear()
    {
        root_ = nullptr;
    }

    // Depth-First Search (Pre-order traversal)
    void dfsTraversal(std::function<void(const T&)> visitFunc) const
    {
        if (root_)
        {
            dfsHelper(root_, visitFunc);
        }
    }

    // Breadth-First Search (Level-order traversal)
    void bfsTraversal(std::function<void(const T&)> visitFunc) const
    {
        if (!root_) return;

        std::queue<std::shared_ptr<TreeNode<T>>> queue;
        queue.push(root_);

        while (!queue.empty())
        {
            auto current = queue.front();
            queue.pop();

            visitFunc(current->data);

            for (const auto& child : current->children)
            {
                queue.push(child);
            }
        }
    }

    // Find node with specific value (DFS)
    std::shared_ptr<TreeNode<T>> find(const T& value) const
    {
        if (!root_) return nullptr;
        return findHelper(root_, value);
    }

    std::shared_ptr<TreeNode<T>> findParent(const T& value) const
    {
        auto node = this->find(value);
        auto parent = node ? node->parent.lock() : nullptr;
        return parent;
    }

    std::vector<std::shared_ptr<TreeNode<T>>> getChildren(const T& value) const
    {
        auto node = this->find(value);
        auto children = node ? node->children : std::vector<std::shared_ptr<TreeNode<T>>>{};
        return children;
    }

    // Get height of the tree
    int getHeight() const
    {
        if (!root_) return -1;
        return getHeightHelper(root_);
    }

    // Get size (total number of nodes)
    size_t getSize() const
    {
        if (!root_) return 0;
        return getSizeHelper(root_);
    }

    // Get all leaf nodes
    std::vector<T> getLeafNodes() const
    {
        std::vector<T> leaves;
        if (root_)
        {
            getLeafNodesHelper(root_, leaves);
        }
        return leaves;
    }

    // Print tree structure
    void printTree() const
    {
        if (root_)
        {
            printTreeHelper(root_, 0);
        }
    }

    // Get path from root to specific node
    std::vector<T> getPathTo(const T& value) const
    {
        if (!root_) return {};

        std::vector<T> path;
        if (getPathToHelper(root_, value, path))
        {
            return path;
        }
        return {};
    }

    // Check if tree contains a value
    bool contains(const T& value) const
    {
        return find(value) != nullptr;
    }

    // Get level of a node (distance from root)
    int getLevel(const T& value) const
    {
        auto path = getPathTo(value);
        return path.empty() ? -1 : static_cast<int>(path.size()) - 1;
    }

    // Get all nodes at specific level
    std::vector<T> getNodesAtLevel(int level) const
    {
        std::vector<T> result;
        if (level < 0 || !root_) return result;

        getNodesAtLevelHelper(root_, 0, level, result);
        return result;
    }

    // Post-order traversal
    void postOrderTraversal(std::function<void(const T&)> visitFunc) const
    {
        if (root_)
        {
            postOrderHelper(root_, visitFunc);
        }
    }

private:
    std::shared_ptr<TreeNode<T>> root_;

    void dfsHelper(const std::shared_ptr<TreeNode<T>>& node, 
                   std::function<void(const T&)> visitFunc) const
    {
        if (node)
        {
            visitFunc(node->data);
            for (const auto& child : node->children)
            {
                dfsHelper(child, visitFunc);
            }
        }
    }

    std::shared_ptr<TreeNode<T>> findHelper(const std::shared_ptr<TreeNode<T>>& node, 
                                           const T& value) const
    {
        if (!node) return nullptr;
        
        if (node->data == value)
        {
            return node;
        }

        for (const auto& child : node->children)
        {
            auto result = findHelper(child, value);
            if (result)
            {
                return result;
            }
        }
        return nullptr;
    }

    int getHeightHelper(const std::shared_ptr<TreeNode<T>>& node) const
    {
        if (!node || node->isLeaf())
        {
            return 0;
        }

        int maxHeight = 0;
        for (const auto& child : node->children)
        {
            maxHeight = std::max(maxHeight, getHeightHelper(child));
        }
        return maxHeight + 1;
    }

    size_t getSizeHelper(const std::shared_ptr<TreeNode<T>>& node) const
    {
        if (!node) return 0;
        
        size_t count = 1; // Count current node
        for (const auto& child : node->children)
        {
            count += getSizeHelper(child);
        }
        return count;
    }

    void getLeafNodesHelper(const std::shared_ptr<TreeNode<T>>& node, 
                           std::vector<T>& leaves) const
    {
        if (node->isLeaf())
        {
            leaves.push_back(node->data);
        }
        else
        {
            for (const auto& child : node->children)
            {
                getLeafNodesHelper(child, leaves);
            }
        }
    }

    void printTreeHelper(const std::shared_ptr<TreeNode<T>>& node, int level) const
    {
        if (!node) return;

        for (int i = 0; i < level; ++i)
        {
            std::cout << "  ";
        }
        std::cout << node->data << std::endl;

        for (const auto& child : node->children)
        {
            printTreeHelper(child, level + 1);
        }
    }

    bool getPathToHelper(const std::shared_ptr<TreeNode<T>>& node, 
                        const T& value, std::vector<T>& path) const
    {
        if (!node) return false;

        path.push_back(node->data);

        if (node->data == value)
        {
            return true;
        }

        for (const auto& child : node->children)
        {
            if (getPathToHelper(child, value, path))
            {
                return true;
            }
        }

        path.pop_back();
        return false;
    }

    void getNodesAtLevelHelper(const std::shared_ptr<TreeNode<T>>& node, 
                              int currentLevel, int targetLevel, 
                              std::vector<T>& result) const
    {
        if (!node) return;

        if (currentLevel == targetLevel)
        {
            result.push_back(node->data);
            return;
        }

        if (currentLevel < targetLevel)
        {
            for (const auto& child : node->children)
            {
                getNodesAtLevelHelper(child, currentLevel + 1, targetLevel, result);
            }
        }
    }

    void postOrderHelper(const std::shared_ptr<TreeNode<T>>& node, 
                        std::function<void(const T&)> visitFunc) const
    {
        if (node)
        {
            for (const auto& child : node->children)
            {
                postOrderHelper(child, visitFunc);
            }
            visitFunc(node->data);
        }
    }
};

#endif // __MULTI_NODE_TREE_HPP__