#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for(int i=0;i<num_nodes;i++)
        {
            Vector2D pos=start+(end-start)*((double)i/((double)num_nodes-1.0));
            masses.push_back(new Mass(pos,node_mass,false));
        }
        for(int i=0;i<num_nodes-1;i++) springs.push_back(new Spring(masses[i],masses[i+1],k));
        //Comment-in this part when you implement the constructor
        //这一块是教程给的
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            //这一块套公式即可
            auto len=(s->m1->position - s->m2->position).norm(); //长度
            s->m1->forces+=-s->k*(s->m1->position - s->m2->position)/len*(len-s->rest_length);
            s->m2->forces+=-s->k*(s->m2->position - s->m1->position) / len * (len - s->rest_length); //与上面方向相反
        }

        for (auto &m : masses)
        {
            float kd = 0.005; // damping coefficient 阻尼系数
            if (!m->pinned)
            {
                // 无阻尼
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                //显示欧拉   下一个位置用当前速度计算得到
                // auto a=m->forces/m->mass+gravity; //加速度
                // m->position+=m->velocity*delta_t;
                // m->velocity+=a*delta_t;
                //半隐式欧拉  使用下一时间的速度计算下一时间的位置
                // auto a=m->forces/m->mass+gravity; //加速度
                // m->velocity+=a*delta_t;
                // m->position+=m->velocity*delta_t;

                //有阻尼
                // TODO (Part 2): Add global damping
                auto a=m->forces/m->mass+gravity-kd*m->velocity/m->mass;
                //这两句不变
                m->velocity+=a*delta_t;
                m->position+=m->velocity*delta_t;
            }   

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            //这里和上面的代码差不多，这一块套公式即可
            auto len=(s->m1->position - s->m2->position).norm(); //长度
            s->m1->forces+=-s->k*(s->m1->position - s->m2->position)/len*(len-s->rest_length);
            s->m2->forces+=-s->k*(s->m2->position - s->m1->position) / len * (len - s->rest_length); //与上面方向相反
        }

        for (auto &m : masses)//这个循环遍历所有质量点，更新它们的位置。
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position; //记录下当前位置
                auto a=m->forces/m->mass+gravity;
                // // TODO (Part 3.1): Set the new position of the rope mass
                // //套公式(无阻尼)
                // m->position=temp_position+(temp_position-m->last_position)+a*delta_t*delta_t;
                // m->last_position=temp_position;
                //加上阻尼
                double damping_factor=0.00005;
                // TODO (Part 4): Add global Verlet damping
                //照着教程敲公式即可
                m->position=temp_position+(1-damping_factor)*(temp_position-m->last_position)+a*delta_t*delta_t;
                m->last_position=temp_position;
            }
            m->forces=Vector2D(0,0);
        }
        
    }
}
