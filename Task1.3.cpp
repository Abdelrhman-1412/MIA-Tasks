#include <iostream>
#include <math.h>

using namespace std;

int main(){
    int n;
    cin>>n;
    int a[n];
    for(int i=0;i<n;i++){
        cin>>a[i];
    }
    int num;
    cin>>num;
    int counter=0;
    for(int i=0;i<n;i++){
        if(a[i]==num){
            cout<<i<<endl;
            break;
        }
        counter++;
    }
    if(counter==n){
        cout<<-1<<endl;
    }
}