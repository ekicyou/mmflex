using System.Text;
using Assimp;
using SlimDX;

namespace MMF.Model.Assimp
{
    public static class AssimpHelper
    {
        public static Vector4 ToSlimDX(this Color4D color)
        {
            return new Vector4(color.R,color.G,color.B,color.A);
        }

        public static Vector4 ToSlimDXVec4(this Vector3D vec,float w=1f)
        {
            return new Vector4(vec.X,vec.Y,vec.Z,w);
        }

        public static Vector3 ToSlimDX(this Vector3D vec)
        {
            return new Vector3(vec.X,vec.Y,vec.Z);
        }

        public static Vector2 ToSlimDXVec2(this Vector3D vec)
        {
            return new Vector2(vec.X,vec.Y);
        }

        public static Vector3 InvZ(this Vector3 source)
        {
            source.Z = -source.Z;
            return source;
        }
        public static Vector4 InvZ(this Vector4 source)
        {
            source.Z = -source.Z;
            return source;
        }

        public static Vector3 InvX(this Vector3 source)
        {
            source.X = -source.X;
            return source;
        }
        public static Vector4 InvX(this Vector4 source)
        {
            source.X = -source.X;
            return source;
        }

        public static string getFilterString(AssimpFileFilter fileFilter)
        {
            StringBuilder builder=new StringBuilder();
            builder.Append("全てのファイル|*.*|");

            if (fileFilter.HasFlag(AssimpFileFilter.CommonModelFile))
            {
                builder.Append("COLLADAファイル|*.dae|");
                builder.Append("Blenderファイル|*.blend|");
                builder.Append("3DS Max 3DSファイル|*.3ds|");
                builder.Append("3DS Max ASEファイル|*.ase|");
                builder.Append("Wavefront Objectファイル|*.obj|");
                builder.Append("IFCファイル|*.ifc|");
                builder.Append("XGLファイル|*.xgl|");
                builder.Append("Auto CAD DXF|*.dxf|");
                builder.Append("LiveWaveファイル|*.lwo|");
                builder.Append("LiveWave Sceneファイル|*.lws|");
                builder.Append("Modoファイル|*.lxo|");
                builder.Append("StereoLithoGraphyファイル|*.stl|");
                builder.Append("DirectX Xファイル|*.x|");
                builder.Append("AC3Dファイル|*.ac|");
                builder.Append("MilkShape3D ファイル|*.ms3d|");
                builder.Append("TrueSpaceファイル|*.cob,*.scn|");
            }
            if (fileFilter.HasFlag(AssimpFileFilter.CommonGameEngineFile))
            {
                builder.Append("Ogre XMLファイル|*.xml|");
                builder.Append("Irrlicht Meshファイル|.irrmesh|");
                builder.Append("Irrlicht Sceneファイル|*.irr|");
            }
            if (fileFilter.HasFlag(AssimpFileFilter.CommonGameFile))
            {
                builder.Append("Quakeファイル|*.mdl,*.md2,*.md3,*.pk3|");
                builder.Append("Return to Castle Wolfenstein|*.mdc|");
                builder.Append("Doom3ファイル|*.md5|");
                builder.Append("ValveModel|*.smd,*.vta|");
                builder.Append("Starcraft II M3|*.m3|");
                builder.Append("Unreal|*.3d|");
            }
            if (fileFilter.HasFlag(AssimpFileFilter.OtherFile))
            {
                builder.Append("BlitzBasic3Dファイル|*.b3d|");
                builder.Append("Quick3Dファイル|*.q3d,*.q3s|");
                builder.Append("Neutral File Format/Sence8ファイル|*.nff|");
                builder.Append("ObjectFileFormatファイル|*.off|");
                builder.Append("PovRAY Rawファイル|*.raw|");
                builder.Append("Terragen Terrainファイル|*.ter|");
                builder.Append("3DGSファイル|*.mdl,*.hmp|");
                builder.Append("Izware Nendoファイル|*.ndo|");

            }
            if(builder.Length>0)builder.Remove(builder.Length - 1, 1);
            return builder.ToString();
        }

    }
}
