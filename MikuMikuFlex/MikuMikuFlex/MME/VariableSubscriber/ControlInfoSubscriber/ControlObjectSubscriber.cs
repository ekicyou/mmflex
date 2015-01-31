using System;
using System.Collections.Generic;
using System.Linq;
using MMF.Bone;
using MMF.Model;
using MMF.Model.PMX;
using MMF.Morph;
using MMF.Utility;
using SlimDX;
using SlimDX.Direct3D11;

namespace MMF.MME.VariableSubscriber.ControlInfoSubscriber
{
    internal sealed class ControlObjectSubscriber : SubscriberBase
    {
        internal ControlObjectSubscriber()
        {
        }

        private ControlObjectSubscriber(VariableType type,string itemName,string name,TargetObject target,bool isSelf)
        {
            variableType = type;
            this.itemName = itemName;
            this.name = name;
            this.target = target;
            this.isSelf = isSelf;
        }

        private VariableType variableType;
        private string itemName;
        private string name;
        private TargetObject target;
        private IDrawable targetDrawable;
        private bool isSelf;

        private static TargetObject[] NeedFloat = {TargetObject.X,TargetObject.Y,TargetObject.Z,TargetObject.Rx,TargetObject.Ry,TargetObject.Rz,TargetObject.Si,TargetObject.Tr,TargetObject.FaceName };
        private static TargetObject[] NeedFloat3 = { TargetObject.XYZ, TargetObject.Rxyz };


        public override string Semantics
        {
            get { return "CONTROLOBJECT"; }
        }

        public override VariableType[] Types
        {
            get { return new[] { VariableType.Bool, VariableType.Float, VariableType.Float3, VariableType.Float4, VariableType.Float4x4 }; }
        }


        public override SubscriberBase GetSubscriberInstance(EffectVariable variable, RenderContext context, MMEEffectManager effectManager, int semanticIndex)
        {
            VariableType type=0;
            TargetObject target = TargetObject.UnUsed;
            string itemName = null;
            string typeName=variable.GetVariableType().Description.TypeName.ToLower();
            switch (typeName)
            {
                case "bool":
                    type = VariableType.Bool;
                    break;
                case "float":
                    type = VariableType.Float;
                    break;
                case "float3":
                    type = VariableType.Float3;
                    break;
                case "float4":
                    type = VariableType.Float4;
                    break;
                case "float4x4":
                    type = VariableType.Float4x4;
                    break;
                default:
                    break;
            }
            EffectVariable nameVariable = EffectParseHelper.getAnnotation(variable, "name", "string");
            if (nameVariable == null)
            {
                throw new InvalidMMEEffectShaderException(
                    string.Format("定義済みセマンティクス「CONTROLOBJECT」の適用されている変数「{0} {1}:CONTROLOBJECT」に対してはアノテーション「string name」は必須ですが、指定されませんでした。", typeName,variable.Description.Name));
            }
            string name = nameVariable.AsString().GetString();
            //Selfの場合はターゲットは自分自身となる
            if (name.ToLower().Equals("(self)"))
            {
                isSelf = true;
            }
            else
            {
                
            }
            //TODO (OffscreenOwner)がnameに指定されたときの対応
            EffectVariable itemVariable = EffectParseHelper.getAnnotation(variable, "item", "string");

            if(itemVariable!=null)
            {
                itemName = itemVariable.AsString().GetString();  
                switch(itemName.ToLower())
                {
                    case "x":
                        target = TargetObject.X;
                        break;
                    case "y":
                        target = TargetObject.Y;
                        break;
                    case "z":
                        target = TargetObject.Z;
                        break;
                    case "xyz":
                        target = TargetObject.XYZ;
                        break;
                    case "rx":
                        target = TargetObject.Rx;
                        break;
                    case "ry":
                        target = TargetObject.Ry;
                        break;
                    case "rz":
                        target = TargetObject.Rz;
                        break;
                    case "rxyz":
                        target = TargetObject.Rxyz;
                        break;
                    case "si":
                        target = TargetObject.Si;
                        break;
                    case "tr":
                        target = TargetObject.Tr;
                        break;
                    default:
                        target = type == VariableType.Float ? TargetObject.FaceName : TargetObject.BoneName;
                        break;
                 }
                if (NeedFloat.Contains(target)&&type!=VariableType.Float)
                {
                    throw new InvalidMMEEffectShaderException(
                        string.Format("定義済みセマンティクス「CONTROLOBJECT」の適用されている変数「{0} {1}:CONTROLOBJECT」にはアノテーション「string item=\"{2}\"」が適用されていますが、「{2}」の場合は「float {1}:CONTROLOBJECT」である必要があります。", typeName, variable.Description.Name,itemName));
                }
                if (NeedFloat3.Contains(target) && type != VariableType.Float3)
                {
                    throw new InvalidMMEEffectShaderException(
                        string.Format("定義済みセマンティクス「CONTROLOBJECT」の適用されている変数「{0} {1}:CONTROLOBJECT」にはアノテーション「string item=\"{2}\"」が適用されていますが、「{2}」の場合は「float3 {1}:CONTROLOBJECT」である必要があります。", typeName, variable.Description.Name, itemName));
                
                }
            }
            return new ControlObjectSubscriber(type, itemName, name, target,isSelf);
        }

        public override void Subscribe(EffectVariable subscribeTo, SubscribeArgument variable)
        {
            var currentModel = variable.Context.CurrentTargetContext.WorldSpace.getDrawableByFileName(name);
            if(currentModel==null)return;
            IDrawable targetDrawable = isSelf ? variable.Model : currentModel;
                if (target == TargetObject.UnUsed)
                {
                    switch (variableType)
                    {
                        case VariableType.Float4x4:
                            subscribeTo.AsMatrix().SetMatrix(variable.Context.MatrixManager.WorldMatrixManager.getWorldMatrix(targetDrawable));
                            break;
                        case VariableType.Float3:
                            subscribeTo.AsVector().Set(targetDrawable.Transformer.Position);
                            break;
                        case VariableType.Float4:
                            subscribeTo.AsVector().Set(new Vector4(targetDrawable.Transformer.Position, 1f));
                            break;
                        case VariableType.Float:
                            subscribeTo.AsScalar().Set(targetDrawable.Transformer.Scale.Length());
                            break;
                        case VariableType.Bool:
                            subscribeTo.AsScalar().Set(targetDrawable.Visibility);
                            break;
                        default:
                            break;
                    }
                }else
                if (target == TargetObject.BoneName)
                {
                    IEnumerable<PMXBone> targetBone = (from bone in ((PMXModel)targetDrawable).Skinning.Bone where bone.BoneName == itemName select bone);
                    foreach (var bone in targetBone)
                    {
                        Matrix mat = bone.GlobalPose * variable.Context.MatrixManager.WorldMatrixManager.getWorldMatrix(targetDrawable);
                        switch (variableType)
                        {
                            case VariableType.Float4x4:
                                subscribeTo.AsMatrix().SetMatrix(mat);
                                break;
                            case VariableType.Float3:
                                subscribeTo.AsVector().Set(Vector3.TransformCoordinate(bone.Position,mat));
                                break;
                            case VariableType.Float4:
                                subscribeTo.AsVector().Set(new Vector4(Vector3.TransformCoordinate(bone.Position, mat),1f));
                                break;
                            default:
                                break;
                        }
                        break;
                    }
            }else
            if (target == TargetObject.FaceName)
            {
                IMorphManager morphManager = ((PMXModel) targetDrawable).Morphmanager;
                subscribeTo.AsScalar().Set(morphManager.getMorphProgress(name));
            }
            else
            {
                switch (target)
                {
                    case TargetObject.X:
                        subscribeTo.AsScalar().Set(targetDrawable.Transformer.Position.X);
                        break;
                    case TargetObject.Y:
                        subscribeTo.AsScalar().Set(targetDrawable.Transformer.Position.Y);
                        break;
                    case TargetObject.Z:
                        subscribeTo.AsScalar().Set(targetDrawable.Transformer.Position.Z);
                        break;
                    case TargetObject.XYZ:
                        subscribeTo.AsVector().Set(targetDrawable.Transformer.Position);
                        break;
                    case TargetObject.Rx:
                    case TargetObject.Ry:
                    case TargetObject.Rz:
                    case TargetObject.Rxyz:
                        float xRotation, yRotation, zRotation; //X,Y,Z軸回転量に変換する。
                                int type = 0; //分解パターン
                                if (
                                    !CGHelper.FactoringQuaternionXYZ(targetDrawable.Transformer.Rotation, out xRotation,
                                        out yRotation, out zRotation))
                                {
                                    if (
                                        !CGHelper.FactoringQuaternionYZX(targetDrawable.Transformer.Rotation, out yRotation,
                                            out zRotation, out xRotation))
                                    {
                                        CGHelper.FactoringQuaternionZXY(targetDrawable.Transformer.Rotation, out zRotation,
                                            out xRotation, out yRotation);
                                        type = 2;
                                    }
                                    else
                                        type = 1;
                                }
                                else type = 0;
                        if (target == TargetObject.Rx)
                        {
                            subscribeTo.AsScalar().Set(xRotation);
                        }else if (target == TargetObject.Ry)
                        {
                            subscribeTo.AsScalar().Set(yRotation);
                        }else if (target == TargetObject.Rz)
                        {
                            subscribeTo.AsScalar().Set(zRotation);
                        }
                        else
                        {
                            subscribeTo.AsVector().Set(new Vector3(xRotation, yRotation, zRotation));
                        }
                        break;
                    case TargetObject.Si:
                        subscribeTo.AsScalar().Set(targetDrawable.Transformer.Scale.Length());
                        break;
                    case TargetObject.Tr:
                        //TODO Trへの対応
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }

        }
    }

        //
}
