#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

int main(int argc, char *argv[]) {
  
  WbNodeRef node; 
  WbFieldRef field;
  int time = -5, currentBot, i, j = 0;
  
  wb_robot_init();
  
    // get the root children field
  const WbNodeRef root_node = wb_supervisor_node_get_root();
  const WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  const int n = wb_supervisor_field_get_count(root_children_field);
  printf("This world contains %d nodes:\n", n);
  WbNodeRef wheelbots[n];
  int botIndex[n];
  
  for (i=0; i<n; i++)
  {
    node = wb_supervisor_field_get_mf_node(root_children_field, i);
    //nodename = wb_supervisor_node_get_def(node);
    printf("found nodeRef [%s]\n", wb_supervisor_node_get_def(node));
    if(strcmp(wb_supervisor_node_get_def(node), "WheelBot") == 0)
    {  
      j+=1;        
      wheelbots[j] = node;
      botIndex[j] = i;
      printf("found WheelBot %d at index %d\n", j, i);     
    }   
  }
  
  node = wb_supervisor_field_get_mf_node(root_children_field, botIndex[1]);
  field = wb_supervisor_node_get_field(node, "radarCrossSection");
  wb_supervisor_field_set_sf_float(field, 0);
  printf("set radarCS to 0 for wheelbot 1\n");
  currentBot = 1;
  
  while (wb_robot_step(32) != -1) 
  {
    float distance = 20;
    float botX = wb_supervisor_node_get_position(wheelbots[currentBot])[0];
    float botZ = wb_supervisor_node_get_position(wheelbots[currentBot])[2];
    for (i = 1; i <= j; i++)
    {
      if (i != currentBot)
      {
        float x = wb_supervisor_node_get_position(wheelbots[i])[0];
        float z = wb_supervisor_node_get_position(wheelbots[i])[2];
        distance = sqrt( (x - botX)*(x - botX) + (z - botZ)*(z - botZ) );
      }
      if(distance < 0.2 && wb_robot_get_time() >= time + 5)
      {
        time = wb_robot_get_time();
        currentBot = i;
        printf("BOT FOUND, time: %d\n", time);
        node = wb_supervisor_field_get_mf_node(root_children_field, botIndex[i]);
        field = wb_supervisor_node_get_field(node, "radarCrossSection");
        wb_supervisor_field_set_sf_float(field, 0);
        printf("set radarCS to 0 for wheelbot %d\n", i);
        for (int k = 1; k <= j; k++)
        {
          if (k != currentBot)
          {
            node = wb_supervisor_field_get_mf_node(root_children_field, botIndex[k]);
            field = wb_supervisor_node_get_field(node, "radarCrossSection");
            wb_supervisor_field_set_sf_float(field, 1);
          }
        }        
      }
    }
    
    
  }
  
  wb_robot_cleanup();

  return 0;
}
