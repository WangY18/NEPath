#include "Connector.h"
#include "Basic.h"
#include "Curve.h"

// Connected Fermat Spiral (CFS)
// Zhao, H., Gu, F., Huang, Q. X., Garcia, J., Chen, Y., Tu, C., ... & Chen, B. (2016). Connected fermat spirals for layered fabrication. ACM Transactions on Graphics, 35(4), 1-10.
path Connector::ConnectedFermatSpiral_MultMinimum(pathnode *root, double dis, double in /*=-1.0*/)
{
    pathnode *real_parent = root->parent;
    root->parent = NULL;

    pathnode *root2 = root;
    while (true)
    { // Find the first fork or leaf
        if (root2->children.size() != 1)
        {
            break;
        }
        root2 = root2->children[0];
    }

    std::stack<pathnode *> S_top;
    std::stack<pathnode *> S_bottom;
    std::stack<double> x_in;
    std::stack<double> y_in;
    if (in >= 0)
    {
        x_in.push(Curve::interp_id(root->data.x, root->data.length, in));
        y_in.push(Curve::interp_id(root->data.y, root->data.length, in));
    }
    S_top.push(root);
    S_bottom.push(root2);

    while (!S_top.empty())
    {
        pathnode *child = S_bottom.top();
        if (child->children.size() > 1)
        { // Branches.
            // Stack branches one by one. Note that children with higher ranks will be added to the stack, so in the end, one can directly pop them herefork.
            for (std::size_t i = 0; i < child->children.size(); ++i)
            {
                pathnode *children = child->children[i];
                while (children->children.size() == 1)
                {
                    children = children->children[0];
                }
                S_top.push(child->children[i]);
                S_bottom.push(children); // Leavf or branching point.
                double id1, id2;
                Curve::curves_nearest(child->data, child->children[i]->data, id1, id2);
                x_in.push(Curve::interp_id(child->data.x, child->data.length, id1));
                y_in.push(Curve::interp_id(child->data.y, child->data.length, id1));
            }
        }
        else
        { // Leaf. Directly apply FermatSpiral_SingleMinimum.
            // pathnode *write = NULL;

            pathnode *parent = S_top.top();
            S_top.pop();
            S_bottom.pop();

            if (!parent->parent)
            {
                if (root == root2)
                { // Prevent repeated calculations in the final step
                    continue;
                }
                double in_now = -1.0;
                if (!x_in.empty())
                {
                    in_now = Curve::nearest_id(parent->data.x, parent->data.y, parent->data.length, x_in.top(), y_in.top());
                    x_in.pop();
                    y_in.pop();
                }
                path p = FermatSpiral_SingleMinimum(parent, dis, in_now);
                pathnode *del_node = child;
                while (del_node != parent)
                {
                    del_node->data.clear_with_delete();
                    del_node = del_node->parent;
                    delete del_node->children[del_node->children.size() - 1];
                    del_node->children[del_node->children.size() - 1] = NULL;
                    del_node->children.pop_back();
                }
                parent->data.clear_with_delete();
                parent->data.length = p.length;
                parent->data.x = p.x;
                parent->data.y = p.y;
                p.clear_without_delete();

                // write = parent;
            }
            else
            {
                pathnode *grandparent = new pathnode();
                pathnode *grandparent_old = parent->parent;
                parent->parent = grandparent;
                grandparent->children.push_back(parent);
                grandparent->data.length = grandparent_old->data.length;
                grandparent->data.x = grandparent_old->data.x;
                grandparent->data.y = grandparent_old->data.y;

                double in_now = Curve::nearest_id(grandparent->data.x, grandparent->data.y, grandparent->data.length, x_in.top(), y_in.top());
                x_in.pop();
                y_in.pop();

                path p = FermatSpiral_SingleMinimum(grandparent, dis, in_now);
                pathnode *del_node = child;
                while (del_node != grandparent)
                {
                    del_node->data.clear_with_delete();
                    del_node = del_node->parent;
                    delete del_node->children[del_node->children.size() - 1];
                    del_node->children[del_node->children.size() - 1] = NULL;
                    del_node->children.pop_back();
                }
                grandparent->data.clear_without_delete();
                delete grandparent;
                grandparent_old->data.clear_with_delete();
                grandparent_old->data.length = p.length;
                grandparent_old->data.x = p.x;
                grandparent_old->data.y = p.y;
                grandparent_old->children[grandparent_old->children.size() - 1] = NULL;
                grandparent_old->children.pop_back();
                p.clear_without_delete();

                // write = grandparent_old;
            }
        }
    }
    root->parent = real_parent;
    return root->data;
}

// CFS with a single local minimum
path Connector::FermatSpiral_SingleMinimum(pathnode *root, double dis, double in /*=-1.0*/)
{
    if (!root->children.size())
    { // Only a level
        return Curve::wash_dis(root->data, dis * 0.2);
    }

    if (in < 0)
    {
        double t;
        Curve::curves_nearest(root->data, root->children[0]->data, in, t);
    }

    double out = in;
    bool out_forward_in = true;
    bool in_run = false;
    bool first_circle = true;
    vector<double> xin;
    vector<double> yin;
    vector<double> xout;
    vector<double> yout;

    while (root)
    {
        bool circle_small = Curve::TotalLength(root->data.x, root->data.y, root->data.length, true) < 2.0 * dis;
        // Adjust the distance between in-point and out-point.
        if (circle_small)
        {
            // This circle is too small. Consider the furthest two points in this circle as in-point and out-point.
            double near_id = 0.5 * (in + out);
            double x_near = Curve::interp_id(root->data.x, root->data.length, near_id);
            double y_near = Curve::interp_id(root->data.y, root->data.length, near_id);
            double far_id = Curve::furthest_id(root->data.x, root->data.y, root->data.length, x_near, y_near);
            double x_far = Curve::interp_id(root->data.x, root->data.length, far_id);
            double y_far = Curve::interp_id(root->data.y, root->data.length, far_id);
            in = ceil(near_id);
            double dism = dis(x_near, y_near, root->data.x[int(in)], root->data.y[int(in)]);
            dism = min(dism, dis(x_far, y_far, root->data.x[int(in)], root->data.y[int(in)]));
            for (int i = (int(in) + 1) % root->data.length; subset_cycle(near_id, far_id, i); i = (i + 1) % root->data.length)
            {
                double disnow = dis(x_near, y_near, root->data.x[i], root->data.y[i]);
                disnow = min(dism, dis(x_far, y_far, root->data.x[i], root->data.y[i]));
                if (disnow > dism)
                {
                    dism = disnow;
                    in = i;
                }
            }
            out = ceil(far_id);
            dism = dis(x_near, y_near, root->data.x[int(out)], root->data.y[int(out)]);
            dism = min(dism, dis(x_far, y_far, root->data.x[int(out)], root->data.y[int(out)]));
            for (int i = (int(out) + 1) % root->data.length; subset_cycle(far_id, near_id, i); i = (i + 1) % root->data.length)
            {
                double disnow = dis(x_near, y_near, root->data.x[i], root->data.y[i]);
                disnow = min(dism, dis(x_far, y_far, root->data.x[i], root->data.y[i]));
                if (disnow > dism)
                {
                    dism = disnow;
                    out = i;
                }
            }

            double lb_io = Curve::LengthBetween(root->data.x, root->data.y, root->data.length, in, out);
            double lb_oi = Curve::LengthBetween(root->data.x, root->data.y, root->data.length, out, in);
            out_forward_in = (lb_io <= lb_oi);
        }
        else
        { // This circle is large enough.
            if (std::abs(in - out) < 1e-6)
            {
                if (!first_circle)
                { // Consider the last upper circle.
                    double x0 = Curve::interp_id(root->data.x, root->data.length, in);
                    double y0 = Curve::interp_id(root->data.y, root->data.length, in);
                    double dis0in = dis(x0, y0, xin[xin.size() - 1], yin[yin.size() - 1]);
                    double dis0out = dis(x0, y0, xout[xout.size() - 1], yout[yout.size() - 1]);
                    double out1 = Curve::ForDis(root->data.x, root->data.y, root->data.length, in, dis);
                    double x1 = Curve::interp_id(root->data.x, root->data.length, out1);
                    double y1 = Curve::interp_id(root->data.y, root->data.length, out1);
                    double dis1in = dis(x1, y1, xin[xin.size() - 1], yin[yin.size() - 1]);
                    double dis1out = dis(x1, y1, xout[xout.size() - 1], yout[yout.size() - 1]);
                    double out2 = Curve::BackDis(root->data.x, root->data.y, root->data.length, in, dis);
                    double x2 = Curve::interp_id(root->data.x, root->data.length, out2);
                    double y2 = Curve::interp_id(root->data.y, root->data.length, out2);
                    double dis2in = dis(x2, y2, xin[xin.size() - 1], yin[yin.size() - 1]);
                    double dis2out = dis(x2, y2, xout[xout.size() - 1], yout[yout.size() - 1]);
                    if (min(dis0in + dis1out, dis1in + dis0out) < min(dis0in + dis2out, dis2in + dis0out))
                    { // The point close the the in-point and out-point in the last circle as out-point in this circle.
                        if (dis0in + dis1out <= dis1in + dis0out)
                        {               // 0-in, 1-out
                            out = out1; // in' = in, out' = fordis(in) = fordis(in') > in'
                            out_forward_in = true;
                        }
                        else
                        {              // 0-out, 1-in
                            out = in;  // out' = in
                            in = out1; // in' = fordis(in) = fordis(out') > out'
                            out_forward_in = false;
                        }
                    }
                    else
                    {
                        if (dis0in + dis2out <= dis2in + dis0out)
                        {               // 0-in, 1-out
                            out = out2; // in' = in, out' = backdis(in) = backdis(in') < in'
                            out_forward_in = false;
                        }
                        else
                        {              // 0-out, 1-in
                            out = in;  // out' = in
                            in = out2; // in' = backdis(in) = backdis(out') < out'
                            out_forward_in = true;
                        }
                    }
                }
                else
                {
                    double out1 = Curve::ForDis(root->data.x, root->data.y, root->data.length, in, dis);
                    double x1 = Curve::interp_id(root->data.x, root->data.length, out1);
                    double y1 = Curve::interp_id(root->data.y, root->data.length, out1);
                    double dis1 = Curve::distance_point2path(root->children[0]->data.x, root->children[0]->data.y,
                                                             root->children[0]->data.length, x1, y1);

                    double out2 = Curve::BackDis(root->data.x, root->data.y, root->data.length, in, dis);
                    double x2 = Curve::interp_id(root->data.x, root->data.length, out2);
                    double y2 = Curve::interp_id(root->data.y, root->data.length, out2);
                    double dis2 = Curve::distance_point2path(root->children[0]->data.x, root->children[0]->data.y,
                                                             root->children[0]->data.length, x2, y2);

                    if (dis1 < dis2)
                    {
                        out = out2;
                        out_forward_in = false;
                    }
                    else
                    {
                        out = out1;
                        out_forward_in = true;
                    }
                }
            }
            else
            {
                double lb_io = Curve::LengthBetween(root->data.x, root->data.y, root->data.length, in, out);
                double lb_oi = Curve::LengthBetween(root->data.x, root->data.y, root->data.length, out, in);
                out_forward_in = (lb_io <= lb_oi);
                double lb = std::min(lb_io, lb_oi);

                double in1, xin0, yin0, xin1, yin1, out1, xout0, yout0, xout1, yout1;

                xin0 = Curve::interp_id(root->data.x, root->data.length, in);
                yin0 = Curve::interp_id(root->data.y, root->data.length, in);
                xout0 = Curve::interp_id(root->data.x, root->data.length, out);
                yout0 = Curve::interp_id(root->data.y, root->data.length, out);

                if (out_forward_in)
                { // out > in
                    in1 = Curve::ForDis(root->data.x, root->data.y, root->data.length, in, lb - dis);
                    xin1 = Curve::interp_id(root->data.x, root->data.length, in1);
                    yin1 = Curve::interp_id(root->data.y, root->data.length, in1);
                    out1 = Curve::BackDis(root->data.x, root->data.y, root->data.length, out, lb - dis);
                    xout1 = Curve::interp_id(root->data.x, root->data.length, out1);
                    yout1 = Curve::interp_id(root->data.y, root->data.length, out1);
                }
                else
                { // in > out
                    in1 = Curve::BackDis(root->data.x, root->data.y, root->data.length, in, lb - dis);
                    xin1 = Curve::interp_id(root->data.x, root->data.length, in1);
                    yin1 = Curve::interp_id(root->data.y, root->data.length, in1);
                    out1 = Curve::ForDis(root->data.x, root->data.y, root->data.length, out, lb - dis);
                    xout1 = Curve::interp_id(root->data.x, root->data.length, out1);
                    yout1 = Curve::interp_id(root->data.y, root->data.length, out1);
                }

                double dis_in0in = dis(xin0, yin0, xin[xin.size() - 1], yin[yin.size() - 1]);
                double dis_in0out = dis(xin0, yin0, xout[xout.size() - 1], yout[yout.size() - 1]);
                double dis_in1in = dis(xin1, yin1, xin[xin.size() - 1], yin[yin.size() - 1]);
                double dis_in1out = dis(xin1, yin1, xout[xout.size() - 1], yout[yout.size() - 1]);
                double dis_out0in = dis(xout0, yout0, xin[xin.size() - 1], yin[yin.size() - 1]);
                double dis_out0out = dis(xout0, yout0, xout[xout.size() - 1], yout[yout.size() - 1]);
                double dis_out1in = dis(xout1, yout1, xin[xin.size() - 1], yin[yin.size() - 1]);
                double dis_out1out = dis(xout1, yout1, xout[xout.size() - 1], yout[yout.size() - 1]);

                if (min(dis_in0in + dis_out1out, dis_in0out + dis_out1in) <=
                    min(dis_in1in + dis_out0out, dis_in1out + dis_out0in))
                {
                    if (dis_in0in + dis_out1out < dis_in0out + dis_out1in)
                    {               // in0-in, out1-out
                        out = out1; // in' = in; out' = backdis(out) > in
                    }
                    else
                    { // in0-out, out1-in
                        out = in;
                        in = out1;
                        out_forward_in = !out_forward_in;
                    }
                }
                else
                {
                    if (dis_in1in + dis_out0out < dis_in1out + dis_out0in)
                    { // in1-in, out0-out
                        in = in1;
                    }
                    else
                    { // in1-out, out0-in
                        in = out;
                        out = in1;
                        out_forward_in = !out_forward_in;
                    }
                }
            }
        }

        if ((!first_circle) && interset(Curve::interp_id(root->data.x, root->data.length, in),
                                        Curve::interp_id(root->data.y, root->data.length, in),
                                        xin[xin.size() - 1], yin[yin.size() - 1],
                                        Curve::interp_id(root->data.x, root->data.length, out),
                                        Curve::interp_id(root->data.y, root->data.length, out),
                                        xout[xout.size() - 1], yout[yout.size() - 1]))
        {
            swap(in, out);
            out_forward_in = !out_forward_in;
        }

        xin.push_back(Curve::interp_id(root->data.x, root->data.length, in));
        yin.push_back(Curve::interp_id(root->data.y, root->data.length, in));
        xout.push_back(Curve::interp_id(root->data.x, root->data.length, out));
        yout.push_back(Curve::interp_id(root->data.y, root->data.length, out));

        if (root->children.size())
        {
            if (circle_small)
            {
                out_forward_in = !out_forward_in;
                continue;
            }
            if (in_run)
            { // Vary in-point��fix out-point.
                if (out_forward_in)
                { // outf > out > in
                    double outf = Curve::ForDis(root->data.x, root->data.y, root->data.length, out, dis);
                    int id_in_begin = floor(in);
                    for (int i = id_in_begin; subset_cycle(out, i, outf, false, false); i = (i + root->data.length - 1) % root->data.length)
                    {
                        xin.push_back(root->data.x[i]);
                        yin.push_back(root->data.y[i]);
                    }
                    xin.push_back(Curve::interp_id(root->data.x, root->data.length, outf));
                    yin.push_back(Curve::interp_id(root->data.y, root->data.length, outf));
                }
                else
                { // in > out > outb
                    double outb = Curve::BackDis(root->data.x, root->data.y, root->data.length, out, dis);
                    int id_in_begin = int(ceil(in)) % root->data.length;
                    for (int i = id_in_begin; subset_cycle(i, out, outb, false, false); i = (i + 1) % root->data.length)
                    {
                        xin.push_back(root->data.x[i]);
                        yin.push_back(root->data.y[i]);
                    }
                    xin.push_back(Curve::interp_id(root->data.x, root->data.length, outb));
                    yin.push_back(Curve::interp_id(root->data.y, root->data.length, outb));
                }
            }
            else
            { // Vary out-point��fix in-point.
                if (out_forward_in)
                { // out > in > inb
                    double inb = Curve::BackDis(root->data.x, root->data.y, root->data.length, in, dis);
                    int id_out_begin = int(ceil(out)) % root->data.length;
                    for (int i = id_out_begin; subset_cycle(i, in, inb, false, false); i = (i + 1) % root->data.length)
                    {
                        xout.push_back(root->data.x[i]);
                        yout.push_back(root->data.y[i]);
                    }
                    xout.push_back(Curve::interp_id(root->data.x, root->data.length, inb));
                    yout.push_back(Curve::interp_id(root->data.y, root->data.length, inb));
                }
                else
                { // inf > in > out
                    double inf = Curve::ForDis(root->data.x, root->data.y, root->data.length, in, dis);
                    int id_out_begin = floor(out);
                    for (int i = id_out_begin; subset_cycle(in, i, inf, false, false); i = (i + root->data.length - 1) % root->data.length)
                    {
                        xout.push_back(root->data.x[i]);
                        yout.push_back(root->data.y[i]);
                    }
                    xout.push_back(Curve::interp_id(root->data.x, root->data.length, inf));
                    yout.push_back(Curve::interp_id(root->data.y, root->data.length, inf));
                }
            }

            in_run = !in_run;
            root = root->children[0];
            in = Curve::nearest_id(root->data.x, root->data.y, root->data.length, xin[xin.size() - 1], yin[yin.size() - 1]);
            out = Curve::nearest_id(root->data.x, root->data.y, root->data.length, xout[xout.size() - 1], yout[yout.size() - 1]);
        }
        else
        { // The last circle
            int idm = 0;
            double dism = min(dis(root->data.x[0], root->data.y[0], xin[xin.size() - 1], yin[yin.size() - 1]),
                              dis(root->data.x[0], root->data.y[0], xout[xout.size() - 1], yout[yout.size() - 1]));
            for (int i = 1; i < root->data.length; ++i)
            {
                double disnow = min(dis(root->data.x[i], root->data.y[i], xin[xin.size() - 1], yin[yin.size() - 1]),
                                    dis(root->data.x[i], root->data.y[i], xout[xout.size() - 1], yout[yout.size() - 1]));
                if (disnow > dism)
                {
                    dism = disnow;
                    idm = i;
                }
            }
            if (subset_cycle(in, out, idm, true, false))
            { // Counter clockwise, idm between in-point and out-point
                for (int i = ceil(in); i != floor(out); i = (i + 1) % root->data.length)
                {
                    xin.push_back(root->data.x[i]);
                    yin.push_back(root->data.y[i]);
                }
            }
            else
            { // Clockwise, idm between out-point and in-point
                for (int i = ceil(out); i != floor(in); i = (i + 1) % root->data.length)
                {
                    xout.push_back(root->data.x[i]);
                    yout.push_back(root->data.y[i]);
                }
            }
            break;
        }
        first_circle = false;
    }
    for (int i = xout.size() - 1; i >= 0; --i)
    {
        xin.push_back(xout[i]);
        yin.push_back(yout[i]);
    }
    return Curve::wash_dis(xin.data(), yin.data(), xin.size(), dis * 0.2, false);
}

// Connect based on Depth First Search.
path Connector::ConnectedDFS(pathnode *root)
{
    stack<pathnode *> S;
    pathnode *real_parent = root->parent;
    root->parent = NULL;
    S.push(root);
    while (!S.empty())
    {
        pathnode *child = S.top();
        if (child->children.size())
        {
            S.push(child->children[child->children.size() - 1]);
            continue;
        }

        S.pop();
        if (!child->parent)
        {
            break;
        }
        pathnode *parent = child->parent;
        double idc = 0.0;
        double idp = 0.0;
        Curve::curves_nearest(parent->data, child->data, idp, idc); // Find a child nearest the parent.

        double *x = new double[parent->data.length + child->data.length + 4];
        double *y = new double[parent->data.length + child->data.length + 4];
        x[0] = Curve::interp_id(parent->data.x, parent->data.length, idp);
        y[0] = Curve::interp_id(parent->data.y, parent->data.length, idp);
        int idp_begin = ((int)ceil(idp)) % parent->data.length;
        for (int i = 0; i < parent->data.length; ++i)
        {
            x[i + 1] = parent->data.x[(idp_begin + i) % parent->data.length];
            y[i + 1] = parent->data.y[(idp_begin + i) % parent->data.length];
        }
        x[parent->data.length + 1] = Curve::interp_id(parent->data.x, parent->data.length, idp);
        y[parent->data.length + 1] = Curve::interp_id(parent->data.y, parent->data.length, idp);

        x[parent->data.length + 2] = Curve::interp_id(child->data.x, child->data.length, idc);
        y[parent->data.length + 2] = Curve::interp_id(child->data.y, child->data.length, idc);
        int idc_begin = ((int)ceil(idc)) % child->data.length;
        for (int i = 0; i < child->data.length; ++i)
        {
            x[i + parent->data.length + 3] = child->data.x[(idc_begin + i) % child->data.length];
            y[i + parent->data.length + 3] = child->data.y[(idc_begin + i) % child->data.length];
        }
        x[child->data.length + parent->data.length + 3] = Curve::interp_id(child->data.x, child->data.length, idc);
        y[child->data.length + parent->data.length + 3] = Curve::interp_id(child->data.y, child->data.length, idc);

        path p(x, y, child->data.length + parent->data.length + 4);
        parent->data.steal(p);
        child->data.clear_with_delete();
        delete child;
        parent->children.pop_back();
    }

    root->parent = real_parent;
    return root->data;
}