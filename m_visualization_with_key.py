class m_visualization_with_key:

        def __init__(self, pts_lst, smooth = 1):
            self.smooth = smooth
            self.pts_lst = self.load_pts(pts_lst)
            #self.pcds = [self.load_pts(pts) for pts in pts_lst]
            #print(self.pcds)
            self.cur_ind = 0


            key_to_callback = {}
            key_to_callback[ord("D")] = self.next
            key_to_callback[ord("A")] = self.prev
            # key_to_callback[ord("Y")] = self.up
            # key_to_callback[ord("U")] = self.down
            # key_to_callback[ord("H")] = self.up
            # key_to_callback[ord("J")] = self.down
            # key_to_callback[ord("C")] = self.both

            o3d.visualization.draw_geometries_with_key_callbacks([self.pts_lst],key_to_callback)
                #[self.pcds[self.cur_ind]], key_to_callback,

            #)


        def load_pts(self, pts):
            print(pts)
            if self.smooth != 1:
                pts = pts[[rnd.random() <= self.smooth for i in range(len(pts))]]

            cur_fn_path = os.path.join('media', 'point_cloud_train', 'test')
            cur_fn = os.path.join(cur_fn_path, 'test.pcd')
            if not os.path.isdir(cur_fn_path):
                os.mkdir(cur_fn_path)

            print(pts)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            # o3d.io.write_point_cloud(cur_fn, pcd)

            # return o3d.io.read_point_cloud(cur_fn)
            return pcd


        def refresh(self, vis):
            vis.clear_geometries()
            vis.add_geometry(self.pcds[self.cur_ind])

        def next(self, vis):
            if self.cur_ind < len(self.pcds) - 1:
                self.cur_ind += 1
            self.refresh(vis)
            return False

        def prev(self, vis):
            if self.cur_ind > 0:
                self.cur_ind -= 1
            self.refresh(vis)
            return False

        def load_all(self, vis):
            vis.clear_geometries()
            vis.add_geometry(self.pcds[0])
            vis.add_geometry(self.pcds[1])


        def up(self, vis):
            self.pts_lst[0][..., 2] += .1
            self.pcds[0] = self.load_pts(self.pts_lst[0])
            self.load_all(vis)
            return False

        def down(self, vis):
            self.pts_lst[0][..., 2] -= .1
            self.pcds[0] = self.load_pts(self.pts_lst[0])
            self.load_all(vis)
            return False


        def left(self, vis):
            self.pts_lst[0][..., 1] += .1
            self.pcds[0] = self.load_pts(self.pts_lst[0])
            self.load_all(vis)
            return False

        def right(self, vis):
            self.pts_lst[0][..., 1] -= .1
            self.pcds[0] = self.load_pts(self.pts_lst[0])
            self.load_all(vis)
            return False


        def both(self, vis):
            self.load_all(vis)
            return False
