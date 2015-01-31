using System;
using System.Collections.Generic;
using System.Linq;
using MMF.Model;
using MMF.Motion;
using MMF.Sprite;

namespace MMF.DeviceManager
{
    public class WorldSpace:IDisposable
    {
        private readonly RenderContext _context;

        public WorldSpace(RenderContext context)
        {
            _context = context;
            drawableGroups.Add(new DrawableGroup(0, "Default", _context));
        }

        public void AddDrawableGroup(DrawableGroup group)
        {
            drawableGroups.Add(group);
            drawableGroups.Sort(new DrawableGroup.DrawableGroupComparer());
        }

        public void RemoveDrawableGroup(string key)
        {
            DrawableGroup removeTarget=null;
            foreach (var drawableGroup in drawableGroups)
            {
                if (drawableGroup.GroupName.Equals(key))
                {
                    removeTarget = drawableGroup;
                    break;
                }
            }
            if (removeTarget == null) return;
            drawableGroups.Remove(removeTarget);
        }

        private List<DrawableGroup> drawableGroups=new List<DrawableGroup>();

        public IReadOnlyList<DrawableGroup> DrawableGroups
        {
            get { return drawableGroups; }
        }

        private List<IMovable> moveResources=new List<IMovable>();
        private List<IDynamicTexture> dynamicTextures=new List<IDynamicTexture>();
        private List<IGroundShadowDrawable> groundShadowDrawables=new List<IGroundShadowDrawable>();
        private List<IEdgeDrawable> edgeDrawables=new List<IEdgeDrawable>();
        private bool isDisposed;

        public List<IMovable> MoveResources
        {
            get { return moveResources; }
            private set { moveResources = value; }
        }

        public List<IDynamicTexture> DynamicTextures
        {
            get { return dynamicTextures; }
            private set { dynamicTextures = value; }
        }

        public List<IGroundShadowDrawable> GroundShadowDrawables
        {
            get { return groundShadowDrawables; }
            private set { groundShadowDrawables = value; }
        }

        public List<IEdgeDrawable> EdgeDrawables
        {
            get { return edgeDrawables; }
            private set { edgeDrawables = value; }
        }

        /// <summary>
        ///     表示するリソースを追加する
        /// </summary>
        /// <param name="drawable"></param>
        public void AddResource(IDrawable drawable,String groupName="Default")
        {
            drawableGroups.First(group=>group.GroupName.Equals(groupName)).AddDrawable(drawable);
            if (drawable is IMovable)
            {
                moveResources.Add((IMovable)drawable);
            }
            if (drawable is IEdgeDrawable)
            {
                edgeDrawables.Add((IEdgeDrawable)drawable);
            }
            if (drawable is IGroundShadowDrawable)
            {
                groundShadowDrawables.Add((IGroundShadowDrawable)drawable);
            }
        }

        /// <summary>
        ///     表示するリソースから削除する
        /// </summary>
        /// <param name="drawable"></param>
        public void RemoveResource(IDrawable drawable)
        {
            foreach (var drawableGroup in drawableGroups)
            {
                if (drawableGroup.DeleteDrawable(drawable))
                {
                    if (drawable is IMovable)
                    {
                        moveResources.Remove((IMovable)drawable);
                    }
                }
            }

        }

        /// <summary>
        /// 登録されているすべての描画の必要があるものを描画します
        /// </summary>
        public void DrawAllResources(TexturedBufferHitChecker hitChecker)
        {
            foreach (var edgeDrawable in edgeDrawables)
            {
                if (edgeDrawable.Visibility) edgeDrawable.DrawEdge();
            }
            foreach (var drawableGroup in drawableGroups)
            {
                drawableGroup.DrawAll();
                _context.BlendingManager.SetBlendState(BlendStateManager.BlendStates.Alignment);
            }
            foreach (var groundShadowDrawable in groundShadowDrawables)
            {
                if (groundShadowDrawable.Visibility) groundShadowDrawable.DrawGroundShadow();
            }
            hitChecker.CheckTarget();
        }

        public IDrawable getDrawableByFileName(string fileName)
        {
            foreach (var drawableGroup in drawableGroups)
            {
                var drawable = drawableGroup.getDrawableByFileName(fileName);
                if (drawable != null) return drawable;
            }
            return null;
        }

        /// <summary>
        /// 更新する動的テクスチャを追加します
        /// </summary>
        /// <param name="dtexture">更新する必要のある動的テクスチャ</param>
        public void AddDynamicTexture(IDynamicTexture dtexture)
        {
            dynamicTextures.Add(dtexture);
        }

        /// <summary>
        /// 動的テクスチャを更新対象から除外します
        /// </summary>
        /// <param name="dtexture">もういらない動的テクスチャ</param>
        public void RemoveDynamicTexture(IDynamicTexture dtexture)
        {
            if (dynamicTextures.Contains(dtexture)) dynamicTextures.Remove(dtexture);
        }


        public void Dispose()
        {
            foreach (var drawableGroup in drawableGroups)
            {
                drawableGroup.Dispose();
            }
            foreach (var dynamicTexture in DynamicTextures)
            {
                if(dynamicTextures!=null)dynamicTexture.Dispose();
            }
            isDisposed = true;
        }

        public bool IsDisposed
        {
            get { return isDisposed; }
        }

        public void UpdateAllDynamicTexture()
        {
            foreach (var dynamicTexture in DynamicTextures)
            {
                dynamicTexture.UpdateTexture();
            }
        }
    }
}
