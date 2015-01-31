using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMDFileParser.MotionParser;
using OpenMMDFormat;
using ProtoBuf;
using SlimDX;
using Header = OpenMMDFormat.Header;

namespace VMD2VME
{
    class Program
    {
        static void Main(string[] args)
        {
            if (args.Length !=2)
            {
                Console.WriteLine("第一引数にvmdファイルへのファイルパス,第二引数に出力先のファイルパスを指定してください。");
                return;
            }
            String filePath = args[0];
            String fileDest = args[1];
            if (!File.Exists(filePath))
            {
                Console.WriteLine("指定されたvmdファイルは存在しません。");
                return;
            }
            using (FileStream source=File.OpenRead(filePath))
            {
                MotionData data = MotionData.getMotion(source);
                //ヘッダー情報の生製
                Console.WriteLine("ヘッダー情報の生成");
                VocaloidMotionEvolved vme=new VocaloidMotionEvolved();
                vme.header=new Header();
                vme.header.versionInfo = "Vocaloid Motion Evolved";
                vme.header.modelInfo.Add(data.header.ModelName);
                //ボーンフレームのインデックス処理
                Console.WriteLine("VME用ボーンインデックスの生成");
                HashSet<String> boneNames=new HashSet<string>();
                foreach (var frameData in data.boneFrameList.boneFrameDatas)
                {
                    boneNames.Add(frameData.BoneName);
                }
                Dictionary<string, ulong> boneIdtable = new Dictionary<string, ulong>();
                ulong id = 0;
                foreach (var boneName in boneNames)
                {
                    boneIdtable.Add(boneName, id);
                    IDTag tag=new IDTag();
                    tag.id = id;
                    tag.name = boneName;
                    vme.boneIDTable.Add(tag);
                    id++;
                }
                //ボーンフレームの書き出し処理
                Console.WriteLine("ボーンフレーム情報の生成");
                foreach (var boneName in boneNames)
                {
                    Console.WriteLine("ボーンフレームテーブル生成:{0}",boneName);
                    BoneFrameTable frameTable=new BoneFrameTable();
                    frameTable.id = boneIdtable[boneName];
                    foreach (var frameData in data.boneFrameList.boneFrameDatas)
                    {
                        if (frameData.BoneName.Equals(boneName))
                        {
                            BoneFrame frame=new BoneFrame();
                            frame.frameNumber = frameData.FrameNumber;
                            frame.position = frameData.BonePosition.ToData();
                            frame.rotation = frameData.BoneRotatingQuaternion.ToData();
                            frame.interpolParameters=new BezInterpolParams();
                            BezInterpolParams interpol = frame.interpolParameters;
                            interpol.X1 = new bvec2()
                            {
                                x = frameData.Interpolation[0][0][0],
                                y = frameData.Interpolation[0][1][0]
                            };
                            interpol.X2 = new bvec2()
                            {
                                x = frameData.Interpolation[0][2][0],
                                y = frameData.Interpolation[0][3][0]
                            };
                            interpol.Y1 = new bvec2()
                            {
                                x = frameData.Interpolation[0][0][1],
                                y = frameData.Interpolation[0][1][1]
                            };
                            interpol.Y2 = new bvec2()
                            {
                                x = frameData.Interpolation[0][2][1],
                                y = frameData.Interpolation[0][3][1]
                            };
                            interpol.Z1 = new bvec2()
                            {
                                x = frameData.Interpolation[0][0][2],
                                y = frameData.Interpolation[0][1][2]
                            };
                            interpol.Z2 = new bvec2()
                            {
                                x = frameData.Interpolation[0][2][2],
                                y = frameData.Interpolation[0][3][2]
                            };
                            interpol.R1 = new bvec2()
                            {
                                x = frameData.Interpolation[0][0][3],
                                y = frameData.Interpolation[0][1][3]
                            };
                            interpol.R2 = new bvec2()
                            {
                                x = frameData.Interpolation[0][2][3],
                                y = frameData.Interpolation[0][3][3]
                            };
                            frameTable.frames.Add(frame);
                        }
                    }
                    frameTable.frames.Sort(new FrameComparator());
                    vme.boneFrameTables.Add(frameTable);
                }
                Console.WriteLine("ボーンフレームリスト生成完了");
                Console.WriteLine("モーフ用フレームリスト生成");
                HashSet<String> faceNames = new HashSet<string>();
                foreach (var frameData in data.morphFrameList.morphFrameDatas)
                {
                    faceNames.Add(frameData.Name);
                }
                Dictionary<string, ulong> faceIdtable = new Dictionary<string, ulong>();
                id = 0;
                foreach (var faceName in faceNames)
                {
                    faceIdtable.Add(faceName, id);
                    IDTag tag = new IDTag();
                    tag.id = id;
                    tag.name = faceName;
                    vme.morphIDTable.Add(tag);
                    id++;
                }
                foreach (var morphName in faceNames)
                {
                    Console.WriteLine("モーフフレームテーブル生成:{0}", morphName);
                    MorphFrameTable frameTable=new MorphFrameTable();
                    frameTable.id = faceIdtable[morphName];
                    foreach (var frameData in data.morphFrameList.morphFrameDatas)
                    {
                        if (frameData.Name.Equals(morphName))
                        {
                            MorphFrame frame=new MorphFrame();
                            frame.frameNumber = frameData.FrameNumber;
                            frame.value = frameData.MorphValue;
                            frameTable.frames.Add(frame);
                        }
                    }
                    frameTable.frames.Sort(new FrameComparator());
                    vme.morphFrameTables.Add(frameTable);
                }
                Console.WriteLine("モーフフレーム生成完了");
                if (data.CameraFrames.CameraFrameCount > 0)
                {
                    Console.WriteLine("カメラ用フレームリスト生成");
                    IDTag camTag = new IDTag();
                    camTag.id = 0;
                    camTag.name = "Default";
                    vme.cameraIDTable.Add(camTag);
                    CameraFrameTable table=new CameraFrameTable();
                    table.id = camTag.id;
                    foreach (var cameraFrame in data.CameraFrames.CameraFrames)
                    {
                        CameraFrame frame=new CameraFrame();
                        frame.frameNumber = cameraFrame.FrameNumber;
                        frame.position = cameraFrame.CameraPosition.ToData();
                        frame.rotation = cameraFrame.CameraRotation.ToData();
                        frame.viewAngle = cameraFrame.ViewAngle;
                        frame.perspective = cameraFrame.Perspective;
                        frame.distance = cameraFrame.Distance;
                        frame.interpolParameters = new BezInterpolParams();
                        BezInterpolParams interpol = frame.interpolParameters;
                        interpol.X1 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[0][0],
                            y = cameraFrame.Interpolation[1][0]
                        };
                        interpol.X2 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[2][0],
                            y = cameraFrame.Interpolation[3][0]
                        };
                        interpol.Y1 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[0][1],
                            y = cameraFrame.Interpolation[1][1]
                        };
                        interpol.Y2 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[2][1],
                            y = cameraFrame.Interpolation[3][1]
                        };
                        interpol.Z1 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[0][2],
                            y = cameraFrame.Interpolation[1][2]
                        };
                        interpol.Z2 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[2][2],
                            y = cameraFrame.Interpolation[3][2]
                        };
                        interpol.R1 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[0][3],
                            y = cameraFrame.Interpolation[1][3]
                        };
                        interpol.R2 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[2][3],
                            y = cameraFrame.Interpolation[3][3]
                        };
                        CameraExtraBezParams camInterpol = frame.cameraInterpolParams=new CameraExtraBezParams();
                        camInterpol.L1 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[0][4],
                            y = cameraFrame.Interpolation[1][4]
                        };
                        camInterpol.L2 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[2][4],
                            y = cameraFrame.Interpolation[3][4]
                        };
                        camInterpol.V1 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[0][5],
                            y = cameraFrame.Interpolation[1][5]
                        };
                        camInterpol.V2 = new bvec2()
                        {
                            x = cameraFrame.Interpolation[2][5],
                            y = cameraFrame.Interpolation[3][5]
                        };

                        table.frames.Add(frame);
                    }
                }
                if (data.LightFrames.LightCount > 0)
                {
                    Console.WriteLine("ライト用フレームリストの生成");
                    IDTag lightTag = new IDTag();
                    lightTag.id = 0;
                    lightTag.name = "Default";
                    vme.lightIDTable.Add(lightTag);
                    LightFrameTable table = new LightFrameTable();
                    table.id = lightTag.id;
                    foreach (var lightFrame in data.LightFrames.LightFrames)
                    {
                        LightFrame frame=new LightFrame();
                        frame.frameNumber = lightFrame.FrameNumber;
                        frame.position = lightFrame.LightPosition.ToData();
                        frame.color = lightFrame.LightColor.ToData();
                        table.frames.Add(frame);
                    }
                }
                if(File.Exists(fileDest))File.Delete(fileDest);
                using (FileStream fs=File.OpenWrite(fileDest))
                {
                    Serializer.Serialize(fs,vme);
                }
            }
        }

    }
}
