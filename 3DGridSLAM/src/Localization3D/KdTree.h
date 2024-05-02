/** 
*@file //TODO 
*  @date 9 Oct 2013 
*  @version 1.0 
*  @brief //TODO. 

This file is part of Luis Garrote Phd work. 
Copyright (C) 2012-2014 Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt) 
All rights reserved.

*
*  @author Luis Garrote (luissgarrote@gmail.com;garrote@isr.uc.pt)
*  @bug No know bugs.
*/ 
#ifndef KDTREE_H
#define KDTREE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

#if defined(WIN32) || defined(__WIN32__)
#include <malloc.h>
#endif

template <class T>
class KdTreeNode{
public:


    double *pos;
    T* data;
    int dir;
    KdTreeNode<T> *left,*right;

};

class kdhyperrect {
public:

    int dim;
    double *min, *max;          /* minimum/maximum coords */
};




template <class T>
class res_node {
public:

    KdTreeNode<T> *item;
    double dist_sq;
    res_node<T> *next;
};

//class kdtree {
//	int dim;
//	KdTreeNode *root;
//	  kdhyperrect *rect;
//	void (*destr)(void*);
//};
template <class T>
class KdTree;

template <class T>
class kdres {
public:

    KdTreeNode<T> *tree;
    res_node<T> *rlist, *riter;
    int size;
};


#define SQ(x)			((x) * (x))
template <class T>
class KdTree{


public:


    int size_;
    int dim;
    KdTreeNode<T> *root;
    kdhyperrect *rect;
    void (*destr)(void*);
    std::vector<T*> members;
    std::vector<T> points;

    bool deleteLater;
    int size(){

        return size_;
    }


    KdTree(int number, bool DeleteLater_=true){

        deleteLater=DeleteLater_;
        free_nodes=NULL;
        rect=NULL;
        root=NULL;
        createTree(number);

    }

    ~KdTree(){

        clear();
        deleteTree(root);
    }


    inline void push_back(T * node){
        double *pos;
        pos=new double[dim];


        if(dim==2){
           pos[0]=(*node).x;
           pos[1]=(*node).y;
        }else if(dim==3){

           pos[0]=(*node).x;
           pos[1]=(*node).y;
           pos[2]=(*node).z;

        }
 
        members.push_back(node);
        points.push_back(*node);
        kd_insert(pos,node);
        delete [] pos;


    }


    T* getNearestNode(T * node,bool ( *foo)(T *node1)){

        double *pos =new double[dim];
        if(dim==2){
           pos[0]=(*node).x;
           pos[1]=(*node).y;
        }else if(dim==3){

           pos[0]=(*node).x;
           pos[1]=(*node).y;
           pos[2]=(*node).z;

        }
 

        kdres<T> *al=kd_nearestCompareFunction(pos,foo);

        delete [] pos;

        if(al->size){
            return  al->riter->item->data;
        }
        return NULL;

    }


    std::vector<T*> getNearestNodes(T *node,double distanceRadius){

        std::vector<T*> output;

        if(size()==0 || node==NULL){
            return output;
        }else{


            double *pos =new double[dim];
        if(dim==2){
           pos[0]=(*node).x;
           pos[1]=(*node).y;
        }else if(dim==3){

           pos[0]=(*node).x;
           pos[1]=(*node).y;
           pos[2]=(*node).z;

        }
 
            kdres<T> *als=kd_nearest_range(pos,distanceRadius);

            if(als!=NULL){
                res_node<T> *element=als->riter;

                while(element!=NULL){
                    output.push_back(element->item->data);
                    element=element->next;
                }
            }
            delete [] pos;
            kd_res_free( als );

            return output;

        }

    }




    T* getNearestNode(T * node){
        double *pos =new double[dim];
        if(dim==2){
           pos[0]=(*node).x;
           pos[1]=(*node).y;
        }else if(dim==3){

           pos[0]=(*node).x;
           pos[1]=(*node).y;
           pos[2]=(*node).z;

        }

        kdres<T> *al=kd_nearest(pos);

        delete [] pos;
        if(al==NULL){
            return NULL;
        }
        if(al->size){
            T* ptx=al->riter->item->data;
            kd_res_free( al );

            return  ptx;
        }
        kd_res_free( al );

        return NULL;
    }


    void getAll(KdTreeNode<T> *element,std::vector<T* > *output){


        if(element==NULL){

            return;
        }


        if(element->data!=NULL){
            output->push_back(element->data);
        }

        getAll(element->left,output);
        getAll(element->right,output);

    }

    void getAllElements(std::vector<T* > *output){

        //        getAll(root,output);

        (*output)=members;

    }
    void clear(){


        if(size()!=0){

            if(deleteLater){
                //                std::vector<T*> values;

                //                getAll(root,&values);

                for(unsigned int i=0;i<members.size();i++){

                    delete members[i];
                }
                members.clear();
            }

            clearTree(root);
            size_=0;
        }
    }


    /* create a kd-tree for "k"-dimensional data */
    bool createTree(int k){

        size_=0;



        dim = k;
        root=0;
        destr = 0;
        rect = 0;

        return true;

    }

    /* free the  kdtree */
    void deleteTree( KdTreeNode<T> *tree){
        if(tree) {
            clearTree(tree);
            delete tree->data;
            free(tree);
        }

    }
    void clear_rec( KdTreeNode<T> *node, void (*destr_)(void*))
    {
        if(!node) return;

        clear_rec(node->left, destr);
        clear_rec(node->right, destr);

        if(destr_) {
            destr_(node->data);
        }
        free(node->pos);
        free(node);
    }


    /* remove all the elements from the tree */
    void clearTree( KdTreeNode<T> *tree){

        clear_rec( tree,  destr);
        root = 0;

        if ( rect) {
            hyperrect_free( rect);
            rect = 0;
        }


    }

    void clearTree(){

        size_=0;
        clear_rec( root,  destr);
        root = 0;

        if ( rect) {
            hyperrect_free( rect);
            rect = 0;
        }


    }

    /* if called with non-null 2nd argument, the function provided
    * will be called on data pointers (see kd_insert) when nodes
    * are to be removed from the tree.
    */

    int insert_rec( KdTreeNode<T> **nptr, const double *pos, T *data, int dir, int dim_)
    {
        int new_dir;
        KdTreeNode<T> *node;

        if(!*nptr) {
            if(!(node = new KdTreeNode<T>())) {
                return -1;
            }
            if(!(node->pos = new double[dim_])) {
                free(node);
                return -1;
            }
            memcpy(node->pos, pos, dim_ * sizeof *node->pos);
            node->data = data;
            node->dir = dir;
            node->left = node->right = 0;
            *nptr = node;
            return 0;
        }

        node = *nptr;
        new_dir = (node->dir + 1) % dim_;
        if(pos[node->dir] < node->pos[node->dir]) {
            return insert_rec(&(*nptr)->left, pos, data, new_dir, dim_);
        }
        return insert_rec(&(*nptr)->right, pos, data, new_dir, dim_);
    }

    int kd_insert( const double *pos, T *data)
    {
        if (insert_rec(& root, pos, data, 0,  dim)) {
            return -1;
        }

        size_=size_+1;

        if ( rect == 0) {
            rect = hyperrect_create( dim, pos, pos);
        } else {
            hyperrect_extend( rect, pos);
        }

        return 0;
    }

    /* insert a node, specifying its position, and optional data */
    int kd_insert3(   double x, double y, double z, void *data)
    {
        double buf[3];
        buf[0] = x;
        buf[1] = y;
        buf[2] = z;
        return kd_insert(buf, data);
    }


    /* Find the nearest node from a given point.
    *
    * This function returns a pointer to a result set with at most one element.
    */
    //  kdres *kd_nearest(KdTreeNode *tree, const double *pos);
    //  kdres *kd_nearestf(KdTreeNode *tree, const float *pos);

    kdres<T> *kd_nearest(  double *pos)
    {
        kdhyperrect *rect1;
        KdTreeNode<T> *result;
        kdres<T> *rset;
        double dist_sq;
        int i;

        if (!root) return 0;
        if (! rect) return 0;

        /* Allocate result set */
        if(!(rset = new kdres<T>())) {
            return 0;
        }
        if(!(rset->rlist = alloc_resnode())) {
            free(rset);
            return 0;
        }
        rset->rlist->next = 0;
        rset->tree = root;

        /* Duplicate the bounding hyperrectangle, we will work on the copy */
        if (!(rect1 = hyperrect_duplicate( rect))) {
            kd_res_free(rset);
            return 0;
        }

        /* Our first guesstimate is the root node */
        result = root;
        dist_sq = 0;
        for (i = 0; i <  dim; i++)
            dist_sq += SQ(result->pos[i] - pos[i]);

        /* Search for the nearest neighbour recursively */
        kd_nearest_i( root, pos, &result, &dist_sq, rect1);

        /* Free the copy of the hyperrect */
        hyperrect_free(rect1);

        /* Store the result */
        if (result) {
            if (rlist_insert(rset->rlist, result, -1.0) == -1) {
                kd_res_free(rset);
                return 0;
            }
            rset->size = 1;
            kd_res_rewind(rset);
            return rset;
        } else {
            kd_res_free(rset);
            return 0;
        }
    }



























    //////
    /// \brief
    /// \param pos
    /// \return
    ///
    ///















    kdres<T> *kd_nearestCompareFunction(  double *pos,bool ( *foo)(T *node1))
    {
        kdhyperrect *rect1;
        KdTreeNode<T> *result;
        kdres<T> *rset;
        double dist_sq;
        int i;

        if (!root) return 0;
        if (! rect) return 0;

        /* Allocate result set */
        if(!(rset = new kdres<T>())) {
            return 0;
        }
        if(!(rset->rlist = alloc_resnode())) {
            free(rset);
            return 0;
        }
        rset->rlist->next = 0;
        rset->tree = root;

        /* Duplicate the bounding hyperrectangle, we will work on the copy */
        if (!(rect1 = hyperrect_duplicate( rect))) {
            kd_res_free(rset);
            return 0;
        }

        /* Our first guesstimate is the root node */
        result = root;
        dist_sq = 0;
        for (i = 0; i <  dim; i++)
            dist_sq += SQ(result->pos[i] - pos[i]);

        /* Search for the nearest neighbour recursively */
        kd_nearest_withCompareFunction( root, pos, &result, &dist_sq, rect1,foo);

        /* Free the copy of the hyperrect */
        hyperrect_free(rect1);

        /* Store the result */
        if (result) {
            if (rlist_insert(rset->rlist, result, -1.0) == -1) {
                kd_res_free(rset);
                return 0;
            }
            rset->size = 1;
            kd_res_rewind(rset);
            return rset;
        } else {
            kd_res_free(rset);
            return 0;
        }
    }



    void kd_nearest_i( KdTreeNode<T> *node, const double *pos,KdTreeNode<T> **result, double *result_dist_sq,  kdhyperrect* rect_)
    {
        int dir = node->dir;
        int i;
        double dummy, dist_sq;
        KdTreeNode<T> *nearer_subtree, *farther_subtree;
        double *nearer_hyperrect_coord, *farther_hyperrect_coord;

        /* Decide whether to go left or right in the tree */
        dummy = pos[dir] - node->pos[dir];
        if (dummy <= 0) {
            nearer_subtree = node->left;
            farther_subtree = node->right;
            nearer_hyperrect_coord = rect_->max + dir;
            farther_hyperrect_coord = rect_->min + dir;
        } else {
            nearer_subtree = node->right;
            farther_subtree = node->left;
            nearer_hyperrect_coord = rect_->min + dir;
            farther_hyperrect_coord = rect_->max + dir;
        }

        if (nearer_subtree) {
            /* Slice the hyperrect to get the hyperrect of the nearer subtree */
            dummy = *nearer_hyperrect_coord;
            *nearer_hyperrect_coord = node->pos[dir];
            /* Recurse down into nearer subtree */
            kd_nearest_i(nearer_subtree, pos, result, result_dist_sq, rect);
            /* Undo the slice */
            *nearer_hyperrect_coord = dummy;
        }

        /* Check the distance of the point at the current node, compare it
      * with our best so far */
        dist_sq = 0;
        for(i=0; i < rect_->dim; i++) {
            dist_sq += SQ(node->pos[i] - pos[i]);
        }
        if (dist_sq < *result_dist_sq) {
            *result = node;
            *result_dist_sq = dist_sq;
        }

        if (farther_subtree) {
            /* Get the hyperrect of the farther subtree */
            dummy = *farther_hyperrect_coord;
            *farther_hyperrect_coord = node->pos[dir];
            /* Check if we have to recurse down by calculating the closest
         * point of the hyperrect and see if it's closer than our
         * minimum distance in result_dist_sq. */
            if (hyperrect_dist_sq(rect_, pos) < *result_dist_sq) {
                /* Recurse down into farther subtree */
                kd_nearest_i(farther_subtree, pos, result, result_dist_sq, rect_);
            }
            /* Undo the slice on the hyperrect */
            *farther_hyperrect_coord = dummy;
        }
    }

    void kd_nearest_withCompareFunction( KdTreeNode<T> *node, const double *pos,KdTreeNode<T> **result, double *result_dist_sq,  kdhyperrect* rect_ ,bool (*foo)(T *node1))
    {
        int dir = node->dir;
        int i;
        double dummy, dist_sq;
        KdTreeNode<T> *nearer_subtree, *farther_subtree;
        double *nearer_hyperrect_coord, *farther_hyperrect_coord;

        /* Decide whether to go left or right in the tree */
        dummy = pos[dir] - node->pos[dir];
        if (dummy <= 0) {
            nearer_subtree = node->left;
            farther_subtree = node->right;
            nearer_hyperrect_coord = rect_->max + dir;
            farther_hyperrect_coord = rect_->min + dir;
        } else {
            nearer_subtree = node->right;
            farther_subtree = node->left;
            nearer_hyperrect_coord = rect_->min + dir;
            farther_hyperrect_coord = rect_->max + dir;
        }

        if (nearer_subtree) {
            /* Slice the hyperrect to get the hyperrect of the nearer subtree */
            dummy = *nearer_hyperrect_coord;
            *nearer_hyperrect_coord = node->pos[dir];
            /* Recurse down into nearer subtree */
            kd_nearest_withCompareFunction(nearer_subtree, pos, result, result_dist_sq, rect,foo);
            /* Undo the slice */
            *nearer_hyperrect_coord = dummy;
        }

        /* Check the distance of the point at the current node, compare it
      * with our best so far */
        dist_sq = 0;
        for(i=0; i < rect_->dim; i++) {
            dist_sq += SQ(node->pos[i] - pos[i]);
        }
        if (dist_sq < *result_dist_sq) {

            if(foo(node->data)){
                *result = node;
                *result_dist_sq = dist_sq;
            }
        }

        if (farther_subtree) {
            /* Get the hyperrect of the farther subtree */
            dummy = *farther_hyperrect_coord;
            *farther_hyperrect_coord = node->pos[dir];
            /* Check if we have to recurse down by calculating the closest
         * point of the hyperrect and see if it's closer than our
         * minimum distance in result_dist_sq. */
            if (hyperrect_dist_sq(rect_, pos) < *result_dist_sq) {
                /* Recurse down into farther subtree */
                kd_nearest_withCompareFunction(farther_subtree, pos, result, result_dist_sq, rect_,foo);
            }
            /* Undo the slice on the hyperrect */
            *farther_hyperrect_coord = dummy;
        }
    }

    kdres<T> *kd_nearest3( double x, double y, double z)
    {
        double pos[3];
        pos[0] = x;
        pos[1] = y;
        pos[2] = z;
        return kd_nearest(root, pos);
    }
    //  kdres *kd_nearest3f(KdTreeNode *tree, float x, float y, float z);

    /* Find the N nearest nodes from a given point.
    *
    * This function returns a pointer to a result set, with at most N elements,
    * which can be manipulated with the kd_res_* functions.
    * The returned pointer can be null as an indication of an error. Otherwise
    * a valid result set is always returned which may contain 0 or more elements.
    * The result set must be deallocated with kd_res_free after use.
    */
    /*
    kdres *kd_nearest_n(  kdtree *tree, const double *pos, int num);
    kdres *kd_nearest_nf(  kdtree *tree, const float *pos, int num);
    kdres *kd_nearest_n3(  kdtree *tree, double x, double y, double z);
    kdres *kd_nearest_n3f(  kdtree *tree, float x, float y, float z);
   */

    /* Find any nearest nodes from a given point within a range.
    *
    * This function returns a pointer to a result set, which can be manipulated
    * by the kd_res_* functions.
    * The returned pointer can be null as an indication of an error. Otherwise
    * a valid result set is always returned which may contain 0 or more elements.
    * The result set must be deallocated with kd_res_free after use.
    */


    res_node<T> *free_nodes;


    res_node<T> *alloc_resnode(void)
    {
        res_node<T> *node;

        //    #ifndef NO_PTHREADS
        //        pthread_mutex_lock(&alloc_mutex);
        //    #endif

        if(!free_nodes) {
            node = new res_node<T>;
        } else {
            node = free_nodes;
            free_nodes = free_nodes->next;
            node->next = 0;
        }

        //    #ifndef NO_PTHREADS
        //        pthread_mutex_unlock(&alloc_mutex);
        //    #endif

        return node;
    }


    void free_resnode(  res_node<T> *node)
    {
        //    #ifndef NO_PTHREADS
        //        pthread_mutex_lock(&alloc_mutex);
        //    #endif

        node->next = free_nodes;
        free_nodes = node;

        //    #ifndef NO_PTHREADS
        //        pthread_mutex_unlock(&alloc_mutex);
        //    #endif
    }


    /* inserts the item. if dist_sq is >= 0, then do an ordered insert */
    /* TODO make the ordering code use heapsort */
    int rlist_insert(  res_node<T> *list,   KdTreeNode<T> *item, double dist_sq)
    {
        res_node<T>*rnode;

        if(!(rnode = alloc_resnode())) {
            return -1;
        }
        rnode->item = item;
        rnode->dist_sq = dist_sq;

        if(dist_sq >= 0.0) {
            while(list->next && list->next->dist_sq < dist_sq) {
                list = list->next;
            }
        }
        rnode->next = list->next;
        list->next = rnode;
        return 0;
    }



    kdres<T> *kd_nearest_range(const double *pos, double range)
    {
        int ret;
        kdres<T> *rset;

        if(!(rset =new kdres<T>())) {
            return 0;
        }
        if(!(rset->rlist = alloc_resnode())) {
            free(rset);
            return 0;
        }
        rset->rlist->next = 0;
        rset->tree = root;

        if((ret = find_nearest( root, pos, range, rset->rlist, 0, dim)) == -1) {
            kd_res_free(rset);
            return 0;
        }
        rset->size = ret;
        kd_res_rewind(rset);
        return rset;
    }

    kdres<T> *kd_nearest_rangeCompareFunction(const double *pos, double range,bool ( *foo)(T *node1))
    {
        int ret;
        kdres<T> *rset;

        if(!(rset =new kdres<T>())) {
            return 0;
        }
        if(!(rset->rlist = alloc_resnode())) {
            free(rset);
            return 0;
        }
        rset->rlist->next = 0;
        rset->tree = root;

        if((ret = find_nearestCompareFunction( root, pos, range, rset->rlist, 0, dim,foo)) == -1) {
            kd_res_free(rset);
            return 0;
        }
        rset->size = ret;
        kd_res_rewind(rset);
        return rset;
    }



    kdres<T> *kd_nearest_range_bynumber(const double *pos, double range,int number)
    {
        int ret;
        kdres<T> *rset;

        if(!(rset =new kdres<T>())) {
            return 0;
        }
        if(!(rset->rlist = alloc_resnode())) {
            free(rset);
            return 0;
        }
        rset->rlist->next = 0;
        rset->tree = root;

        int count=0;

        if((ret = find_nearest( root, pos, range, rset->rlist, 0, dim,number,&count)) == -1) {
            kd_res_free(rset);
            return 0;
        }
        rset->size = ret;
        kd_res_rewind(rset);
        return rset;
    }

    int find_nearest( KdTreeNode<T> *node, const double *pos, double range,   res_node<T> *list, int ordered, int dim_)
    {
        double dist_sq, dx;
        int i, ret, added_res = 0;

        if(!node) return 0;

        dist_sq = 0;
        for(i=0; i<dim_; i++) {
            dist_sq += SQ(node->pos[i] - pos[i]);
        }
        if(dist_sq <= SQ(range)) {
            if(rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
                return -1;
            }
            added_res = 1;
        }

        dx = pos[node->dir] - node->pos[node->dir];

        ret = find_nearest(dx <= 0.0 ? node->left : node->right, pos, range, list, ordered, dim_);
        if(ret >= 0 && fabs(dx) < range) {
            added_res += ret;
            ret = find_nearest(dx <= 0.0 ? node->right : node->left, pos, range, list, ordered, dim_);
        }
        if(ret == -1) {
            return -1;
        }
        added_res += ret;

        return added_res;
    }

    int find_nearestCompareFunction( KdTreeNode<T> *node, const double *pos, double range,   res_node<T> *list, int ordered, int dim_,bool ( *foo)(T *node1))
    {
        double dist_sq, dx;
        int i, ret, added_res = 0;

        if(!node) return 0;



        dist_sq = 0;
        for(i=0; i<dim_; i++) {
            dist_sq += SQ(node->pos[i] - pos[i]);
        }
        if(dist_sq <= SQ(range)) {
            if(foo(node->data)){


                if(rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
                    return -1;
                }
                added_res = 1;
            }
        }

        dx = pos[node->dir] - node->pos[node->dir];

        ret = find_nearestCompareFunction(dx <= 0.0 ? node->left : node->right, pos, range, list, ordered, dim_,foo);
        if(ret >= 0 && fabs(dx) < range) {
            added_res += ret;
            ret = find_nearestCompareFunction(dx <= 0.0 ? node->right : node->left, pos, range, list, ordered, dim_,foo);
        }
        if(ret == -1) {
            return -1;
        }
        added_res += ret;

        return added_res;
    }
    int find_nearest( KdTreeNode<T> *node, const double *pos, double range,   res_node<T> *list, int ordered, int dim_,int number,int *counter)
    {


        if(*counter>=number){

            return 0;
        }

        double dist_sq, dx;
        int i, ret, added_res = 0;

        if(!node) return 0;

        dist_sq = 0;
        for(i=0; i<dim_; i++) {
            dist_sq += SQ(node->pos[i] - pos[i]);
        }
        if(dist_sq <= SQ(range)) {
            if(rlist_insert(list, node, ordered ? dist_sq : -1.0) == -1) {
                return -1;
            }
            added_res = 1;
            *counter=*counter+1;
        }

        dx = pos[node->dir] - node->pos[node->dir];

        ret = find_nearest(dx <= 0.0 ? node->left : node->right, pos, range, list, ordered, dim_,number,counter);
        if(ret >= 0 && fabs(dx) < range) {
            added_res += ret;
            ret = find_nearest(dx <= 0.0 ? node->right : node->left, pos, range, list, ordered, dim_,number,counter);
        }
        if(ret == -1) {
            return -1;
        }

        added_res += ret;

        // std::cout<<"Counter -> "<<*counter<<std::endl;


        return added_res;
    }


    kdres<T> *kd_nearest_range3(   double x, double y, double z, double range)
    {
        double buf[3];
        buf[0] = x;
        buf[1] = y;
        buf[2] = z;
        return kd_nearest_range(root, buf, range);
    }



    /* frees a result set returned by kd_nearest_range() */
    void kd_res_free(  kdres<T> *rset)
    {
        clear_results(rset);
        free_resnode(rset->rlist);
        free(rset);
    }


    /* returns the size of the result set (in elements) */
    int kd_res_size(  kdres<T> *set)
    {
        return (set->size);
    }

    /* rewinds the result set iterator */
    void kd_res_rewind(  kdres<T> *rset)
    {
        rset->riter = rset->rlist->next;
    }

    /* returns non-zero if the set iterator reached the end after the last element */
    int kd_res_end(  kdres<T> *rset)
    {
        return rset->riter == 0;
    }

    /* advances the result set iterator, returns non-zero on success, zero if
    * there are no more elements in the result set.
    */
    int kd_res_next(  kdres<T> *rset)
    {
        rset->riter = rset->riter->next;
        return rset->riter != 0;
    }

    /* returns the data pointer (can be null) of the current result set item
    * and optionally sets its position to the pointers(s) if not null.
    */
    //   void *kd_res_item(  kdres *set, double *pos);
    //   void *kd_res_itemf(  kdres *set, float *pos);
    //   void *kd_res_item3(  kdres *set, double *x, double *y, double *z);
    //   void *kd_res_item3f(  kdres *set, float *x, float *y, float *z);

    //   /* equivalent to kd_res_item(set, 0) */
    //   void *kd_res_item_data(  kdres *set);

private:
    kdhyperrect* hyperrect_create(int dim_, const double *min, const double *max)
    {
        size_t size_s = dim_ * sizeof(double);
        kdhyperrect* rect_ = 0;

        if (!(rect_ = new kdhyperrect)) {
            return 0;
        }

        rect_->dim = dim_;
        if (!(rect_->min = new double[dim_])) {
            free(rect_);
            return 0;
        }
        if (!(rect_->max =  new double[dim_])) {
            free(rect_->min);
            free(rect_);
            return 0;
        }
        memcpy(rect_->min, min, size_s);
        memcpy(rect_->max, max, size_s);

        return rect_;
    }

    void hyperrect_free(  kdhyperrect *rect_)
    {
        free(rect_->min);
        free(rect_->max);
        free(rect_);
    }

    kdhyperrect* hyperrect_duplicate(const  kdhyperrect *rect_)
    {
        return hyperrect_create(rect_->dim, rect_->min, rect_->max);
    }

    void hyperrect_extend(  kdhyperrect *rect_, const double *pos)
    {
        int i;

        for (i=0; i < rect_->dim; i++) {
            if (pos[i] < rect_->min[i]) {
                rect_->min[i] = pos[i];
            }
            if (pos[i] > rect_->max[i]) {
                rect_->max[i] = pos[i];
            }
        }
    }

    double hyperrect_dist_sq(  kdhyperrect *rect_, const double *pos)
    {
        int i;
        double result = 0;
        for (i=0; i < rect_->dim; i++) {
            if (pos[i] < rect_->min[i]) {
                result += SQ(rect_->min[i] - pos[i]);
            } else if (pos[i] > rect_->max[i]) {
                result += SQ(rect_->max[i] - pos[i]);
            }
        }
        return result;
    }


    void clear_results(  kdres<T> *rset)
    {
        res_node<T> *tmp, *node = rset->rlist->next;

        while(node) {
            tmp = node;
            node = node->next;
            free_resnode(tmp);
        }

        rset->rlist->next = 0;
    }




};


#endif // KDTREE_H
