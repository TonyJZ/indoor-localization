#ifndef _DBSCAN_H_Tony_23_AUG_2016_
#define _DBSCAN_H_Tony_23_AUG_2016_

/* Copyright 2015 Gagarine Yaikhom (MIT License) */
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <flann/flann.h>

using namespace std;

namespace DBS
{

#define UNCLASSIFIED -1
#define NOISE 0

#define CORE_POINT 1
#define NOT_CORE_POINT 0

#define SUCCESS 0
#define FAILURE -3


struct point_s 
{
	float x, y, z;
	int cluster_id;
};
typedef struct point_s point_t;

typedef struct node_s node_t;
struct node_s 
{
	unsigned int index;
	node_t *next;
};



struct epsilon_neighbours_s
{
	unsigned int num_members;
	node_t *head, *tail;
};
typedef struct epsilon_neighbours_s epsilon_neighbours_t;

node_t *create_node(unsigned int index);

int append_at_end(
	unsigned int index,
	epsilon_neighbours_t *en);

epsilon_neighbours_t *get_epsilon_neighbours(
	unsigned int index,
	point_t *points,
	unsigned int num_points,
	double epsilon,
	double (*dist)(point_t *a, point_t *b));

void print_epsilon_neighbours(
	point_t *points,
	epsilon_neighbours_t *en);

void destroy_epsilon_neighbours(epsilon_neighbours_t *en);

int expand(
	unsigned int index,
	unsigned int cluster_id,
	point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_t *a, point_t *b));

int spread(
	unsigned int index,
	epsilon_neighbours_t *seeds,
	unsigned int cluster_id,
	point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_t *a, point_t *b));

double euclidean_dist(point_t *a, point_t *b);

double adjacent_intensity_dist(point_t *a, point_t *b);


node_t *create_node(unsigned int index)
{
	node_t *n = (node_t *) calloc(1, sizeof(node_t));
	if (n == NULL)
		perror("Failed to allocate node.");
	else {
		n->index = index;
		n->next = NULL;
	}
	return n;
}

int append_at_end(
	unsigned int index,
	epsilon_neighbours_t *en)
{
	node_t *n = create_node(index);
	if (n == NULL) {
		free(en);
		return FAILURE;
	}
	if (en->head == NULL) {
		en->head = n;
		en->tail = n;
	} else {
		en->tail->next = n;
		en->tail = n;
	}
	++(en->num_members);
	return SUCCESS;
}

static flann_index_t searches;  //trees of images
static struct FLANNParameters p;

epsilon_neighbours_t *get_epsilon_neighbours(
	unsigned int index,
	point_t *points,
	unsigned int num_points,
	double epsilon,
	double (*dist)(point_t *a, point_t *b))
{
	epsilon_neighbours_t *en = (epsilon_neighbours_t *)
		calloc(1, sizeof(epsilon_neighbours_t));
	if (en == NULL) {
		perror("Failed to allocate epsilon neighbours.");
		return en;
	}

	int indices[64];
	float dists[64];
	int max_nn = 32;
	//float radius = 0.1;
	int nDuplicate = 0;
	float a_query[3];   //notice: be consistent with dim 

	a_query[0] = points[index].x;
	a_query[1] = points[index].y;
	a_query[2] = points[index].z;

	int nn = flann_radius_search(searches, a_query, indices, dists, max_nn, epsilon, &p);
	for (int i = 0; i < nn; i++)
	{
		if (index == indices[i])
			continue;

		if (append_at_end(indices[i], en) == FAILURE) 
		{
			destroy_epsilon_neighbours(en);
			en = NULL;
			break;
		}
	}

// 	for (int i = 0; i < num_points; ++i) {
// 		if (i == index)
// 			continue;
// 		if (dist(&points[index], &points[i]) > epsilon)
// 			continue;
// 		else {
// 			if (append_at_end(i, en) == FAILURE) {
// 				destroy_epsilon_neighbours(en);
// 				en = NULL;
// 				break;
// 			}
// 		}
// 	}
	return en;
}

void print_epsilon_neighbours(
	point_t *points,
	epsilon_neighbours_t *en)
{
	if (en) {
		node_t *h = en->head;
		while (h) {
			printf("(%lfm, %lf, %lf)\n",
				points[h->index].x,
				points[h->index].y,
				points[h->index].z);
			h = h->next;
		}
	}
}

void destroy_epsilon_neighbours(epsilon_neighbours_t *en)
{
	if (en) {
		node_t *t, *h = en->head;
		while (h) {
			t = h->next;
			free(h);
			h = t;
		}
		free(en);
	}
}

int expand(
	unsigned int index,
	unsigned int cluster_id,
	point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_t *a, point_t *b))
{
	int return_value = NOT_CORE_POINT;

	epsilon_neighbours_t *seeds =
		get_epsilon_neighbours(index, points,
		num_points, epsilon,
		dist);

	if (seeds == NULL)
		return FAILURE;

	if (seeds->num_members < minpts)
		points[index].cluster_id = NOISE;
	else 
	{
		points[index].cluster_id = cluster_id;
		node_t *h = seeds->head;
		while (h) 
		{
			points[h->index].cluster_id = cluster_id;
			h = h->next;
		}

		h = seeds->head;
		while (h) 
		{
			spread(h->index, seeds, cluster_id, points,
				num_points, epsilon, minpts, dist);
			h = h->next;
		}

		return_value = CORE_POINT;
	}

	destroy_epsilon_neighbours(seeds);
	return return_value;
}

int spread(
	unsigned int index,
	epsilon_neighbours_t *seeds,
	unsigned int cluster_id,
	point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_t *a, point_t *b))
{
	epsilon_neighbours_t *spread =
		get_epsilon_neighbours(index, points,
		num_points, epsilon,
		dist);
	if (spread == NULL)
		return FAILURE;
	if (spread->num_members >= minpts) {
		node_t *n = spread->head;
		point_t *d;
		while (n) {
			d = &points[n->index];
			if (d->cluster_id == NOISE ||
				d->cluster_id == UNCLASSIFIED) {
					if (d->cluster_id == UNCLASSIFIED) {
						if (append_at_end(n->index, seeds)
							== FAILURE) {
								destroy_epsilon_neighbours(spread);
								return FAILURE;
						}
					}
					d->cluster_id = cluster_id;
			}
			n = n->next;
		}
	}

	destroy_epsilon_neighbours(spread);
	return SUCCESS;
}

double euclidean_dist(point_t *a, point_t *b)
{
	return sqrt(pow(a->x - b->x, 2) +
		pow(a->y - b->y, 2) +
		pow(a->z - b->z, 2));
}


void dbscan(
	point_t *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_t *a, point_t *b))
{
	unsigned int i, cluster_id = 1;

	//加快搜索速度
	
	
	float* dataset = NULL;
	float speedup;


	p = DEFAULT_FLANN_PARAMETERS;
	p.algorithm = FLANN_INDEX_KDTREE;
	p.trees = 8;
	p.log_level = FLANN_LOG_INFO;
	p.checks = 64;
	p.cores = 8;

	int dim = 3; //(x,y,z)
	dataset = new float[num_points*dim];

	cout << "build kd-tree for points..." << endl;
	for (int i = 0; i < num_points; ++i)
	{//为每张影像建立kd-tree
		float *ptr = dataset + i*dim;
		
		*ptr = points[i].x; ptr++;
		*ptr = points[i].y; ptr++;
		*ptr = points[i].z; ptr++;
	}

	searches = flann_build_index(dataset, num_points, dim, &speedup, &p);
	cout << " kd-tree is done with " << num_points << " points!" << endl;
// 	cout << endl;
// 	cout << "build tree done!" << endl;


	for (i = 0; i < num_points; ++i)
	{
		if (points[i].cluster_id == UNCLASSIFIED)
		{
			if (expand(i, cluster_id, points,
				num_points, epsilon, minpts,
				dist) == CORE_POINT)

				++cluster_id;
		}
	}

	//delete flann
	if (dataset)	delete[] dataset; dataset = NULL;
	
	flann_free_index(searches, &p);

	return;
}

}

#endif