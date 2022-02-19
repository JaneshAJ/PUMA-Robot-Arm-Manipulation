#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <iostream>
#include "RrtConConBase.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>

//Parameters to tweak for Avoidance of exhaustive nodes
//Limit for detection of exhaustive nodes
#ifndef EX_LIM_BOOL
#define EX_LIM_BOOL 1
#endif

#ifndef EX_LIM
#define EX_LIM 3
#endif

//K-nearest neighbor to choose from
//Boolean if activate or not
#ifndef K_NEAR_BOOL
#define K_NEAR_BOOL 0
#endif

//Factor k
#ifndef K_NEAR
#define K_NEAR 2
#endif

//Simplified task-adapted metric
//bool to activate
#ifndef STAM
#define STAM 1
#endif

//First joint to deactivate
#ifndef SKIP_A
#define SKIP_A 4
#endif

//second joint to deactivate
#ifndef SKIP_B
#define SKIP_B 5
#endif

//Restricting Sample Space
//Boolean to activate
#ifndef RSS
#define RSS 1
#endif

//Radius of the sphere to include samples
#ifndef RSS_RADIUS_END
#define RSS_RADIUS_END 1.8
#endif

//Gaus_Sampling
//Boolean to activate
#ifndef GAUSS_SAMPLE
#define GAUSS_SAMPLE 0
#endif

//EXPLORATION_BIAS
//Boolean to activate
#ifndef EX_BIAS
#define EX_BIAS 0
#endif

YourPlanner::YourPlanner() :
        RrtConConBase(),
        swap_trees(true),
        grow_from_start(true)
{
}

YourPlanner::~YourPlanner()
{
}
::std::string
YourPlanner::getName() const
{
  return "Your Planner";
}


void
YourPlanner::choose(::rl::math::Vector& chosen)
{
   if(RSS == 1){
	::rl::math::Vector try_chosen;
	::rl::math::Real distance_end = 0;
	::rl::math::Real distance_right = 0;
	//While loop to sample as long as the new sample is not within the sphere
	do{
		if(GAUSS_SAMPLE==1){
			try_chosen = this->sampler->generateCollisionFree();
		}
		else {
			try_chosen = RrtConConBase::sampler->generate();
		}
		distance_end = calc_distance(try_chosen, end_q);

	}while((distance_end > RSS_RADIUS_END));
	chosen = try_chosen;
   }
   else{
	if(GAUSS_SAMPLE==1){
			chosen = this->sampler->generateCollisionFree();
	}
	else {
			chosen = RrtConConBase::sampler->generate();
	}
   }
}

::rl::math::Real YourPlanner::calc_distance(const ::rl::math::Vector q1, const ::rl::math::Vector q2)
{
    ::rl::math::Real d =0;
    //Checking if Simplified Task-Adapted Metric should be applied
    if(STAM == 1){ //should be applied
	int jump = 0;
	::rl::math::Vector q1_stam = q1;
	for (::std::size_t l = 0; l < 6; ++l){
		//setting respective joints to same value, such that they do not contribute to distance
		if(l== SKIP_A || l== SKIP_B){ q1_stam(l) = q2(l); }
	}
	//calc the distance
	d = RrtConConBase::model->transformedDistance(q1_stam, q2);
    }
    else {
	d = RrtConConBase::model->transformedDistance(q2, q1);
    }
    return d;
}

void YourPlanner::find_k_nearest(Tree& tree, const ::rl::math::Vector chosen, std::pair<RrtConConBase::Vertex, ::rl::math::Real> *k_nearest, int k)
{
   std::pair<RrtConConBase::Vertex, ::rl::math::Real> all_vertices [num_vertices(tree)];
  //Copied from RrtConConBase, loop through all vertices and store distances
  int counter = 0;	
  //loop through all vertices to store distance
  for (RrtConConBase::VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    ::rl::math::Vector tree_q = *tree[*i.first].q;
    ::rl::math::Real d = calc_distance(chosen, tree_q);
    all_vertices[counter].first = *i.first;
    all_vertices[counter].second = d;
    counter++; 
  }

  //Extract k_nearest and store in array
  for(int i=0; i<=k; i++){
	::rl::math::Real largest_d = 0;
	for (int j = 0;j<num_vertices(tree);j++)
  	{
		if(all_vertices[j].second > largest_d){
			largest_d = all_vertices[j].second;
			k_nearest[i].first = all_vertices[j].first;
			k_nearest[i].second =	RrtConConBase::model->inverseOfTransformedDistance(all_vertices[j].second);
			all_vertices[j].second = 0;
		}
	}
  }
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
   RrtConConBase::Vertex check_vert;
   //Implementation of http://homepages.laas.fr/jcortes/Papers/icra07paper.pdf
   //1. Using one of the k_nearest neighbors to try for connection
   //2. counting if neighbor vertex could be connected, if exceeds Max. number of times --> remove vertex
   Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());
   if(K_NEAR_BOOL == 1){
	   //Find k-nearest Neighbor and choose random
	p.second = 0;
	int k = 0;
	if(num_vertices(tree) > K_NEAR+1){
		k = K_NEAR;
		std::pair<RrtConConBase::Vertex, ::rl::math::Real> k_nearest [k];
	   	find_k_nearest(tree, chosen, k_nearest, k);
	   	k = rand() % k;
	   	p.first = k_nearest[k].first;
	   	p.second = (::rl::math::Real) k_nearest[k].second;
	}
	if( p.second == 0){
		p = RrtConConBase::nearest(tree, chosen);
		//std::cout<<"K_nearest failed. k:"<<k<<" K_NEAR:"<<K_NEAR<<" NumV:"<<num_vertices(tree)<<std::endl;
	}
	//Get newly connected(?) Vertex, use random k_nearest neighbor
	check_vert = RrtConConBase::connect(tree, p, chosen);

    }
    else {
	p = RrtConConBase::nearest(tree, chosen);
	check_vert = RrtConConBase::connect(tree,p, chosen); 
    }

    if(EX_LIM_BOOL == 1){
	    //Check if vertex could have been connected
	    if (check_vert == NULL)
	    {
		//Check if nearest vertex is exhaustive, but do not remove vertices, already connected
		if((tree[p.first].counter >= EX_LIM)&&(tree[p.first].already_connected == false))  {
			//Remove Vertex
			RrtConConBase::Vertex v = p.first;
			remove_vertex(v, tree);
			//std::cout<<"removed vertex"<<std::endl;
			return NULL;
		}
		//Increase counter 
		else { tree[p.first].counter++;}
	    }
	    //Vertex was sucessfully expanded
	    else{ 
		tree[p.first].counter = 0;
		tree[p.first].already_connected = true;
	    }
    }

    return check_vert;
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
    /* Copied from RrtConConBase */
    this->time = ::std::chrono::steady_clock::now();
    // Define the roots of both trees
    this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared<::rl::math::Vector>(*this->start));
    this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared<::rl::math::Vector>(*this->goal));

    Tree *a = &this->tree[0];
    Tree *b = &this->tree[1];

    // Grow tree from the end position if grow_from_start is false
    using ::std::swap;
    if (!grow_from_start) {
        swap(a,b);
    }

    ::rl::math::Vector chosen(this->model->getDof());

    //init sphere (have to adjust for reverse problem)
    end_q = *tree[1][begin[1]].q;
    end_q(1) += 1;

   this->viewer->drawSphere(end_q,RSS_RADIUS_END);

    while ((::std::chrono::steady_clock::now() - this->time) < this->duration) {
        //First grow tree a and then try to connect b.
        //then swap roles: first grow tree b and connect to a.
        for (::std::size_t j = 0; j < 2; ++j) {
	    if(EX_BIAS ==1){
		    if(j==0 && this->getNumVertices()%10==0){
			chosen=*this->goal;
		    }
		    if(j==1 && this->getNumVertices()%10==0){
			chosen=*this->start;
		    }
		    else{
			this->choose(chosen);
		    }
	    }
	    else {
            	this->choose(chosen);
	    }
            //Find the nearest neighbour in the tree
            Neighbor aNearest = this->nearest(*a, chosen);

            //Do a CONNECT step from the nearest neighbour to the sample
            Vertex aConnected = this->connect(*a, aNearest, chosen);

            //If a new node was inserted tree a
            if (NULL != aConnected) {
                // Try a CONNECT step form the other tree to the sample
                Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
                Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

                if (NULL != bConnected) {
                    //Test if we could connect both trees with each other
                    if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q)) {
                        this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
                        this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
                        return true;
                    }
                }
            }

            //Swap the roles of a and b
            if(swap_trees){
                swap(a, b);
            }
        }

    }

    return false;
}

