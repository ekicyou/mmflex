using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.Model;

namespace MMF.DeviceManager
{
    public class DrawableGroup:IDisposable
    {
        public class DrawableGroupComparer : IComparer<DrawableGroup>
        {
            public int Compare(DrawableGroup x, DrawableGroup y)
            {
                return x.priorty - y.priorty;
            }
        }

        private List<IDrawable> drawables=new List<IDrawable>();

        protected int priorty;

        private string groupName;
        protected readonly RenderContext _context;

        public DrawableGroup(int priorty, string groupName,RenderContext context)
        {
            this.priorty = priorty;
            this.groupName = groupName;
            _context = context;
        }

        public string GroupName
        {
            get { return groupName; }
        }

        public void AddDrawable(IDrawable drawable)
        {
            drawables.Add(drawable);
        }

        public bool DeleteDrawable(IDrawable drawable)
        {
            if (drawables.Contains(drawable))
            {
                drawables.Remove(drawable);
                return true;
            }
            return false;
        }

        public void ForEach(Action<IDrawable> act)
        {
            foreach (var drawable in drawables)
            {
                act(drawable);
            }
        }

        public void DrawAll()
        {
            PreDraw();
            foreach (var drawable in drawables)
            {
                if(drawable.Visibility)drawable.Draw();
            }
            PostDraw();
        }

        protected virtual void PostDraw()
        {
            
        }

        protected virtual void PreDraw()
        {
            
        }

        public IDrawable getDrawableByFileName(string fileName)
        {
            return drawables.FirstOrDefault(drawable => drawable.FileName.Equals(fileName));
        }


        public int CompareTo(DrawableGroup other)
        {
            return priorty - other.priorty;
        }

        public void Dispose()
        {
            foreach (var drawable in drawables)
            {
                drawable.Dispose();
            }
        }
    }
}
