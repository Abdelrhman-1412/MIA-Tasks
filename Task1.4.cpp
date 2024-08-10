#include<bits/stdc++.h>
using namespace std;

int main(){
    int i,j;
    cin>>i;
    cin>>j;
    int justice_league[i][j];
    int villains[i][j];
    //declaration of justice league power
    for(int r=0;r<i;r++){
        for(int c=0; c<j;c++){
            cin>>justice_league[r][c];
        }
    }
    //declaration of villains power
    for(int x=0;x<i;x++){
        for(int y=0; y<j;y++){
            cin>>villains[x][y];
        
        }
    }
    //comparisons between power
    int ju=0,vi=0;
    for(int g=0;g<i;g++){
        for(int h=0;h<j;h++){
            if (justice_league[g][h]> villains[g][h]){
                ju++;
            }else if(justice_league[g][h]< villains[g][h]){
                vi++;
            }                 
        }   
    }
    //result of comparison
    if(ju==vi){
        cout<< "Tie";

            }else if (ju>vi){
                cout<<"Justice League";
            }else{
                cout<< "Villains";
            }
}