using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MMF.DeviceManager;
using MMF.Model.Controller.ControllerComponent;
using MMF.Model.Shape;
using SlimDX;

namespace MMF.Model.Controller
{
    public class TransformController:IDrawable,ILockableController
    {
        public void ResetTransformedEventHandler()
        {
            Transformed= delegate { };
        }

        private readonly TexturedBufferHitChecker _hitChecker;

        private Action<TransformController, TransformChangedEventArgs> transformChanged;

        /// <summary>
        /// コントロールにより変更された際に呼び出す。
        /// </summary>
        public class TransformChangedEventArgs:EventArgs
        {
            private IDrawable targetModel;

            private TransformType type;

            public TransformChangedEventArgs(IDrawable targetModel, TransformType type)
            {
                this.targetModel = targetModel;
                this.type = type;
            }

            public IDrawable TargetModel
            {
                get { return targetModel; }
            }

            public TransformType Type
            {
                get { return type; }
            }
        }

        [Flags]
        public enum TransformType
        {
           Translation=0x01,
           Rotation=0x02,
           Scaling=0x04,
           All=0x07
        }

        public event EventHandler<TransformChangedEventArgs> Transformed = delegate { };

        public event EventHandler TargetModelChanged = delegate { }; 

        private List<IHitTestable> controllers=new List<IHitTestable>(); 

        private RotateRingController xRotater;

        private RotateRingController yRotater;

        private RotateRingController zRotater;

        private TranslaterConeController xTranslater;

        private TranslaterConeController yTranslater;

        private TranslaterConeController zTranslater;

        private ScalingCubeController scaler;

        private CenterCrossLine cross;

        public float TransformRotationSensibility { get; set; }

        public float TransformTranslationSensibility { get; set; }

        public float TransformScalingSensibility { get; set; }

        public TransformController(RenderContext context,TexturedBufferHitChecker hitChecker)
        {
            _hitChecker = hitChecker;
            _type=TransformType.All;
            TransformRotationSensibility = 1.0f;
            TransformScalingSensibility = 1.0f;
            TransformTranslationSensibility = 1.0f;
            Transformer=new BasicTransformer();
            //回転コントロール
            xRotater = new RotateRingController(context, this,new Vector4(1, 0, 0, 0.7f), new Vector4(1, 1, 0, 0.7f),
    new SilinderShape.SilinderShapeDescription(0.02f, 30));
            yRotater = new RotateRingController(context,this, new Vector4(0, 1, 0, 0.7f), new Vector4(1, 1, 0, 0.7f),
    new SilinderShape.SilinderShapeDescription(0.02f, 30));
            zRotater = new RotateRingController(context,this, new Vector4(0, 0, 1, 0.7f), new Vector4(1, 1, 0, 0.7f),
    new SilinderShape.SilinderShapeDescription(0.02f, 30));
            cross=new CenterCrossLine(context);
            xRotater.Transformer.Rotation *= Quaternion.RotationAxis(Vector3.UnitZ, (float)(Math.PI / 2));
            zRotater.Transformer.Rotation *= Quaternion.RotationAxis(-Vector3.UnitX, -(float)(Math.PI / 2));
            xRotater.Transformer.Scale = yRotater.Transformer.Scale = zRotater.Transformer.Scale = new Vector3(1f, 0.1f, 1f) * 20;
            xRotater.Transformer.Scale *= 0.998f;
            zRotater.Transformer.Scale *= 0.990f;
            xRotater.Initialize();
            yRotater.Initialize();
            zRotater.Initialize();
            xRotater.OnRotated += RotationChanged;
            yRotater.OnRotated += RotationChanged;
            zRotater.OnRotated += RotationChanged;
            controllers.Add(xRotater);

            controllers.Add(yRotater);
            controllers.Add(zRotater);
            //平衡移動コントロール
            xTranslater=new TranslaterConeController(context,this,new Vector4(1,0,0,0.7f),new Vector4(1,1,0,0.7f));
            yTranslater = new TranslaterConeController(context,this, new Vector4(0, 1, 0, 0.7f), new Vector4(1, 1, 0, 0.7f));
            zTranslater = new TranslaterConeController(context,this, new Vector4(0, 0, 1, 0.7f), new Vector4(1, 1, 0, 0.7f));
            xTranslater.Initialize();
            yTranslater.Initialize();
            zTranslater.Initialize();
            xTranslater.Transformer.Scale =
                yTranslater.Transformer.Scale = zTranslater.Transformer.Scale = new Vector3(2f);
            xTranslater.Transformer.Rotation *= Quaternion.RotationAxis(Vector3.UnitZ, (float)(Math.PI / 2));
            zTranslater.Transformer.Rotation *= Quaternion.RotationAxis(-Vector3.UnitX, -(float)(Math.PI / 2));
            MoveTranslater(xTranslater);
            MoveTranslater(yTranslater);
            MoveTranslater(zTranslater);
            xTranslater.OnTranslated += OnTranslated;
            yTranslater.OnTranslated += OnTranslated;
            zTranslater.OnTranslated += OnTranslated;
            controllers.Add(xTranslater);
            controllers.Add(yTranslater);
            controllers.Add(zTranslater);
            scaler = new ScalingCubeController(context, this, new Vector4(0, 1, 1, 0.7f), new Vector4(1, 1, 0, 1));
            scaler.Initialize();
            scaler.Transformer.Scale=new Vector3(3);
            scaler.OnScalingChanged += OnScaling;
            controllers.Add(scaler);
            hitChecker.CheckTargets.AddRange(controllers);
            Visibility = true;
        }

        private void OnScaling(object sender, ScalingCubeController.ScalingChangedEventArgs e)
        {
            float delta = e.Delta*TransformScalingSensibility;
            sumScaling *= delta;
            if (targetModel != null)
            {
                targetModel.Transformer.Scale += new Vector3(delta);
                Transformed(this,new TransformChangedEventArgs(targetModel,TransformType.Scaling));
                if (transformChanged != null)
                    transformChanged(this, new TransformChangedEventArgs(targetModel, TransformType.Scaling));
            }
        }

        private void OnTranslated(object sender, TranslaterConeController.TranslatedEventArgs e)
        {
            Vector3 trans = e.Translation;
            setTranslationProperty(trans);
        }

        private void MoveTranslater(TranslaterConeController translater)
        {
            translater.Transformer.Position += translater.Transformer.Top*30;
        }

        private void RotationChanged(object sender, RotateRingController.RotationChangedEventArgs e)
        {
            var quat=Quaternion.RotationAxis(e.Axis, -e.Length/100f);
            setRotationProperty(quat);
        }

        private void setRotationProperty(Quaternion quat)
        {
            
            quat = Quaternion.Lerp(Quaternion.Identity, quat, TransformRotationSensibility);
            sumRotation *= quat;
            Transformer.Rotation *= quat;
            xRotater.Transformer.Rotation *= quat;
            yRotater.Transformer.Rotation *= quat;
            zRotater.Transformer.Rotation *= quat;
            if (targetModel != null)
            {
                targetModel.Transformer.Rotation *= quat;
                Transformed(this,new TransformChangedEventArgs(targetModel,TransformType.Rotation));
                if (transformChanged != null)
                    transformChanged(this, new TransformChangedEventArgs(targetModel, TransformType.Rotation));
            }
        }

        private void setTranslationProperty(Vector3 trans)
        {
            trans = Vector3.Lerp(Vector3.Zero, trans, TransformTranslationSensibility);
            sumTranslation += trans;
            foreach (var hitTestable in controllers)
            {
                hitTestable.Transformer.Position += trans;
            }
            cross.AddTranslation(trans);
            if (targetModel != null)
            {
                targetModel.Transformer.Position += trans;
                Transformed(this,new TransformChangedEventArgs(targetModel,TransformType.Translation));
                if (transformChanged != null)
                    transformChanged(this, new TransformChangedEventArgs(targetModel, TransformType.Translation));
            }
        }

        public void Dispose()
        {
            foreach (var hitTestable in controllers)
            {
                _hitChecker.CheckTargets.Remove(hitTestable);
            }
            xRotater.Dispose();
            yRotater.Dispose();
            zRotater.Dispose();
            xTranslater.Dispose();
            yTranslater.Dispose();
            zTranslater.Dispose();
            scaler.Dispose();
            cross.Dispose();
        }

        public TransformType Type
        {
            get { return _type; }
            set
            {
                _type = value;
                if (_type.HasFlag(TransformType.Rotation))
                {
                    xRotater.Visibility = yRotater.Visibility = zRotater.Visibility = true;
                }
                else
                {
                    xRotater.Visibility = yRotater.Visibility = zRotater.Visibility = false;
                }
                if(_type.HasFlag(TransformType.Translation))
                {
                    xTranslater.Visibility = yTranslater.Visibility = zTranslater.Visibility = true;
                }
                else
                {
                    xTranslater.Visibility = yTranslater.Visibility = zTranslater.Visibility = false;
                }
                if (_type.HasFlag(TransformType.Scaling))
                {
                    scaler.Visibility = true;
                }
                else
                {
                    scaler.Visibility = false;
                }
            }
        }

        public void setTargetModel(IDrawable drawable,ITransformer transformer=null, Action<TransformController, TransformChangedEventArgs> del=null)
        {
            transformChanged = del;
            var isChanged = drawable != targetModel;
            targetModel = null;
            setRotationProperty(Quaternion.Invert(sumRotation));
            setTranslationProperty(-sumTranslation);
            if(drawable==null) return;var trans = transformer == null ? drawable.Transformer : transformer;
            setRotationProperty(trans.Rotation);
            setTranslationProperty(trans.Position);
            targetModel = drawable;
            if (isChanged) TargetModelChanged(this, new EventArgs());
        }

        public IDrawable targetModel { get; private set; }

        public bool Visibility { get; set; }
        public string FileName { get; private set; }
        public int SubsetCount { get; private set; }
        public int VertexCount { get; private set; }
        public ITransformer Transformer { get; private set; }
        public void Draw()
        {
            if(targetModel==null||!targetModel.Visibility)return;
            cross.Draw();
            foreach (var hitTestable in controllers)
            {
                if(hitTestable.Visibility)hitTestable.Draw();
            }
            
        }

        public void Update()
        {
            
        }

        private Quaternion sumRotation=Quaternion.Identity;
        private Vector3 sumTranslation;
        private float sumScaling;
        private TransformType _type;

        public Vector4 SelfShadowColor { get; set; }
        public Vector4 GroundShadowColor { get; set; }
        public bool IsLocked { get; set; }

        public void Selected()
        {
            
        }
    }
}
