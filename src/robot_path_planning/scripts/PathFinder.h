#ifndef PATHFINDER_H
#define PATHFINDER_H


#include <string>
#include <vector>



class PathFinder {
  private:
    static const int size  {16} ;
    std::vector<int>path_found;
    std::vector<std::vector<int>>all_possible;


    int map[size][size]= {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,0,1,1,1,1,1,1,0,0},
    {0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0},
    {0,1,1,1,1,1,1,0,1,1,1,1,1,1,0,0},
    {0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0},
    {0,1,1,1,1,1,1,0,1,1,1,1,1,1,0,0},
    {0,1,0,0,0,0,1,0,1,0,0,0,0,1,0,0},
    {0,1,1,1,1,1,1,0,1,1,1,1,1,1,0,0},
    {0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
    int merwe7[size][size]= {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0},
    {0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0},
    {0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,0},
    {0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0},
    {0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,0},
    {0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0},
    {0,1,1,1,1,1,0,1,0,1,1,1,1,1,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

    int clone_map[size][size] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};


    int path_finding(int xi,int yi,int xf,int yf,int map[16][16]);
    void clear_clone_map();

  public:
    PathFinder();
    std::string findPath(int xi, int yi, int xf, int yf, int direction);
};

PathFinder::PathFinder() = default ;


int PathFinder::path_finding(int xi,int yi,int xf,int yf,int map[16][16]){

  	if (map[xi][yi]==0) return 0;

  	if ( (map[xi][yi]==1) && (this->clone_map[xi][yi])==0) {
  		this->path_found.push_back(yi-7);
  		this->path_found.push_back(14-xi);
      this->clone_map[xi][yi]=1;

  	}

  	if ( (xi==xf) && (yi==yf) ) {
      this->all_possible.push_back(this->path_found);
      this->path_found.pop_back();
      this->path_found.pop_back();

      this->clone_map[xi][yi]=0;
  		return 1;
    }

    if ( (this->clone_map[xi][yi+1]==0) && path_finding(xi,yi+1,xf,yf,map)) {

      if (path_finding(xi+1,yi,xf,yf,map) || path_finding(xi-1,yi,xf,yf,map) || path_finding(xi,yi-1,xf,yf,map) ){
        this->clone_map[xi][yi]=0;
        return 1;
      }
      this->clone_map[xi][yi]=0;
  	  return 0;
    }

  	if ( (this->clone_map[xi+1][yi]==0) && path_finding(xi+1,yi,xf,yf,map) ){
  		if ( path_finding(xi,yi+1,xf,yf,map) || path_finding(xi-1,yi,xf,yf,map) || path_finding(xi,yi-1,xf,yf,map)) {
        this->clone_map[xi][yi]=0;
        return 1;
      }
      this->clone_map[xi][yi]=0;
  	return 0;
    }
  	if ( (this->clone_map[xi-1][yi]==0) && path_finding(xi-1,yi,xf,yf,map)) {
  		if (path_finding(xi,yi+1,xf,yf,map) || path_finding(xi+1,yi,xf,yf,map) || path_finding(xi,yi-1,xf,yf,map)){
        this->clone_map[xi][yi]=0;
        return 1;
      }
      this->clone_map[xi][yi]=0;
      return 0;
    }
  	if ( (this->clone_map[xi][yi-1]==0) && path_finding(xi,yi-1,xf,yf,map)) {
  	if (path_finding(xi-1,yi,xf,yf,map) || path_finding(xi+1,yi,xf,yf,map) || path_finding(xi,yi+1,xf,yf,map)){
     this->clone_map[xi][yi]=0;
    return 1 ;
    }
    this->clone_map[xi][yi]=0;
    return 0;
    }
    else {
    this->clone_map[xi][yi]=0;
    this->path_found.pop_back();
    this->path_found.pop_back();

    return 0;
    }
}
void PathFinder::clear_clone_map(){
  for (int i = 0; i <size; i++)
  {
      for (int j=0;j<size;j++)
        this->clone_map[i][j]=0;

  }
}
std::string PathFinder::findPath(int xi, int yi, int xf, int yf, int direction){

    this->path_found.clear();
    this->all_possible.clear();
    clear_clone_map();

    if(direction)
      path_finding(xi,yi,xf,yf,map);
    else if(!direction)
      path_finding(xi,yi,xf,yf,merwe7);

    int l=0;
    for(int i=0;i<this->all_possible.size();i++){
       if ( this->all_possible[i].size ()<this->all_possible[l].size())
        l=i;

    }

    std::string s;

    for (int i=0;i<all_possible[l].size();i+=2){
        s=s+(std::to_string(this->all_possible[l][i]));
        s=s+",";
        s=s+(std::to_string(this->all_possible[l][i+1]));
        s=s+"/";
    }

    return s;

}


#endif
